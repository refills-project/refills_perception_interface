from __future__ import print_function, division

from copy import deepcopy

import PyKDL as kdl
import json
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from robosherlock_msgs.srv import RSQueryService, RSQueryServiceRequest
from rospy import ROSException

from rospy_message_converter import message_converter

from giskardpy.tfwrapper import pose_to_kdl
from knowrob_refills.knowrob_wrapper import KnowRob
from refills_perception_interface.barcode_detection import BarcodeDetector
from refills_perception_interface.not_hacks import add_bottom_layer_if_not_present
from refills_perception_interface.separator_detection import SeparatorClustering
from refills_perception_interface.tfwrapper import transform_pose
from refills_perception_interface.utils import print_with_prefix, error_with_refix

MAP = 'map'


class DetectedObject(object):
    def __init__(self,
                 answer=None,
                 depth=0,
                 width=0,
                 height=0,
                 camera_T_object=None):
        self.map_T_object = None
        if answer is None:
            self.depth = depth
            self.width = width
            self.height = height
            self.camera_T_object = camera_T_object
        else:
            geometry = answer['rs.annotation.Geometry'][0]
            bounding_box = geometry['boundingBox']['rs.pcl.BoundingBox3D']
            self.depth = bounding_box['depth']
            self.width = bounding_box['width']
            self.height = bounding_box['height']
            self.volume = bounding_box['volume']
            pose = answer['rs.annotation.PoseAnnotation'][0]['camera']['rs.tf.StampedPose']
            self.camera_T_object = PoseStamped()
            self.camera_T_object.header.frame_id = pose['frame']
            seconds = int(str(pose['timestamp'])[:10])
            nano_seconds = int(str(pose['timestamp'])[10:])
            self.camera_T_object.header.stamp = rospy.Time(seconds, nano_seconds)
            self.camera_T_object.pose.position = Point(*pose['translation'])
            self.camera_T_object.pose.orientation = Quaternion(*pose['rotation'])
        self.front_area = self.width * self.height

    def fix_dimensions(self, dim1, dim2, dim3):
        return # fuck it
        dims = [dim1, dim2, dim3]
        self.height = dims[abs(np.array(dims) - self.height).argmin()]
        dims.remove(self.height)
        self.width = dims[abs(np.array(dims) - self.width).argmin()]
        dims.remove(self.width)
        self.depth = dims[0]
        camera_T_object = pose_to_kdl(self.camera_T_object.pose)
        object_T_offset = kdl.Frame()
        # object_T_offset.p.x



class FakeRoboSherlock(object):
    prefix = 'robosherlock wrapper'

    def __init__(self, knowrob, num_of_facings=2):
        self.knowrob = knowrob  # type: KnowRob
        self.number_of_facings = num_of_facings
        self.get_all_barcodes()

    def print_with_prefix(self, msg):
        print_with_prefix(msg, self.prefix)

    def error_with_prefix(self, msg):
        error_with_refix(msg, self.prefix)

    def get_all_barcodes(self):
        self.barcodes = set(self.knowrob.get_all_product_dan())

    def start_separator_detection(self, floor_id):
        """
        :type floor_id: str
        """
        self.number_of_facings = max(4, int(np.random.normal(loc=5, scale=2)))
        # self.current_shelf_layer_id = floor_id

    def stop_separator_detection(self, frame_id):
        """
        :type frame_id: str
        :return: list of pose stamps
        :rtype: list
        """
        separators = []
        width = self.knowrob.get_shelf_layer_width(self.current_shelf_layer_id)
        for i in range(self.number_of_facings + 1):
            separator = PoseStamped()
            separator.header.frame_id = self.knowrob.get_perceived_frame_id(self.current_shelf_layer_id)
            x = (i / (self.number_of_facings)) * width
            if x > 0.01 and x < 0.99:
                x = max(0, min(width, x + np.random.normal(scale=0.02)))
            separator.pose.position = Point(x, 0, 0)
            separator.pose.orientation.w = 1
            separator = transform_pose('map', separator)
            separators.append(separator)
        return separators

    def start_barcode_detection(self, floor_id):
        """
        :type floor_id: str
        """
        self.current_shelf_layer_id = floor_id

    def make_rnd_barcode(self):
        while True:
            barcode = '{0:06}'.format(int(np.random.rand() * 1000000))
            if barcode not in self.barcodes:
                return barcode

    def stop_barcode_detection(self, frame_id):
        """
        :type frame_id: str
        :return: dict mapping barcode to pose stamped
        :rtype: dict
        """
        barcodes = {}
        width = self.knowrob.get_shelf_layer_width(self.current_shelf_layer_id)
        # num_of_barcodes = 1
        num_of_barcodes = max(1, self.number_of_facings - int(np.random.rand() * 3))
        for i in range(num_of_barcodes):
            barcode = PoseStamped()
            barcode.header.frame_id = self.knowrob.get_perceived_frame_id(self.current_shelf_layer_id)
            x = max(0,
                    min(width, ((i + .5) / (num_of_barcodes)) * width + np.random.normal(scale=.1 / num_of_barcodes)))
            barcode.pose.position = Point(x, 0, 0)
            barcode.pose.orientation.w = 1
            barcode = transform_pose('map', barcode)
            try:
                if np.random.choice([True]*9+[False]):
                    barcodes[str(self.barcodes.pop())] = barcode
                else:
                    rnd_barcode = self.make_rnd_barcode()
                    barcodes[rnd_barcode] = barcode
            except KeyError:
                self.get_all_barcodes()
                barcodes[self.barcodes.pop()] = barcode
        return barcodes

    def count_product(self, facing_id):
        """
        :type facing_id: str
        :rtype: int
        """
        # self.knowrob.assert_confidence(facing_id, 0.88)
        i = int(np.random.random() * 2)
        if i > 0:
            return 1, None, None
        return 0, None, None

    def start_detect_shelf_layers(self, shelf_system_id):
        """
        :type shelf_system_id: str
        """
        pass

    def stop_detect_shelf_layers(self, shelf_system_id, max_height=1.1):
        """
        :return: list of shelf layer heights
        :rtype: list
        """
        # shelf_system_height = self.knowrob.get_shelf_system_height(shelf_system_id)

        num_of_layer = min(5, max(3, int(np.random.normal(loc=4, scale=0.5))))
        # num_of_layer = 1
        heights = np.array([(x / num_of_layer) * max_height for x in range(num_of_layer)] + [max_height]) + 0.2
        for i in range(len(heights)):
            heights[i] += np.random.normal(scale=0.03)
        heights[0] = min(0.2, heights[0])
        # shelf_system_height = 1
        detected_shelf_layers = heights
        return add_bottom_layer_if_not_present(detected_shelf_layers.tolist(), shelf_system_id, self.knowrob)

    def see(self, depth, width, height, object_type):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position = Point(2.7, 1.9, 0.8)
        p.pose.orientation.w = 1
        return p

    def set_ring_light(self, value=True):
        pass


class RoboSherlock(FakeRoboSherlock):
    def __init__(self, knowrob, name='RoboSherlock', check_camera=True):
        from iai_ringlight_msgs.srv import iai_ringlight_in, iai_ringlight_inRequest
        self.check_camera = check_camera
        self.knowrob = knowrob  # type: KnowRob
        # TODO camera topics as ros param
        self.wait_for_rgb_camera()
        self.wait_for_realsense()
        self.separator_detection = SeparatorClustering(knowrob)
        self.barcode_detection = BarcodeDetector(knowrob)

        self.robosherlock_srv_name = rospy.get_param('~robosherlock_srv_name', '/{}/query'.format(name))
        self.ring_light_srv = rospy.ServiceProxy('iai_ringlight_controller', iai_ringlight_in)

        self.wait_for_robosherlock()

    def set_ring_light(self, value=True):
        from iai_ringlight_msgs.srv import iai_ringlight_in, iai_ringlight_inRequest
        rospy.loginfo('calling ring light switch')
        if value:
            req = iai_ringlight_inRequest(a=9)
        else:
            req = iai_ringlight_inRequest(a=0)
        r = None
        try:
            r = self.ring_light_srv.call(req)
        except:
            self.print_with_prefix('ring_light_switch not available')
        self.print_with_prefix('ring light switch returned {}'.format(r))

    def wait_for_robosherlock(self):
        self.print_with_prefix('waiting for RoboSherlock')
        try:
            rospy.wait_for_service(self.robosherlock_srv_name, timeout=30)
        except ROSException as e:
            self.error_with_prefix('robosherlock unavailable ({})'.format(self.robosherlock_srv_name))
            raise e
        self.robosherlock_service = rospy.ServiceProxy(self.robosherlock_srv_name, RSQueryService)
        self.print_with_prefix('connected to RoboSherlock')

    def wait_for_rgb_camera(self):
        if self.check_camera:
            self.rgb_topic = rospy.get_param('~rgb_topic')
            self.print_with_prefix('waiting for rgb camera')
            try:
                rospy.wait_for_message(self.rgb_topic, rospy.AnyMsg, timeout=20)
            except ROSException as e:
                self.error_with_prefix('rgb camera unavailable ({})'.format(self.rgb_topic))
                raise e
            self.print_with_prefix('rgb camera found')

    def wait_for_realsense(self):
        if self.check_camera:
            self.depth_topic = rospy.get_param('~realsense_topic')
            self.print_with_prefix('waiting for realsense camera')
            try:
                rospy.wait_for_message(self.depth_topic, rospy.AnyMsg, timeout=5)
            except ROSException as e:
                self.error_with_prefix('depth camera unavailable ({})'.format(self.depth_topic))
                raise e
            self.print_with_prefix('realsense camera found')

    def start_separator_detection(self, floor_id):
        self.set_ring_light(True)
        self.separator_detection.start_listening_separators(floor_id)

    def stop_separator_detection(self, frame_id):
        return self.separator_detection.stop_listening()

    def add_separators(self, separators, ys):
        s = separators[0]
        for y in ys:
            new_s = deepcopy(s) #type: PoseStamped
            new_s.pose.position.y += y
            separators.append(new_s)
        separators = sorted(separators, key=lambda x: x.pose.position.z)
        return separators

    def add_layers(self, layers, heights):
        layers.extend(heights)
        return sorted(layers)

    def start_barcode_detection(self, floor_id):
        self.set_ring_light(True)
        self.barcode_detection.start_listening(floor_id)
        pass

    def stop_barcode_detection(self, frame_id):
        return self.barcode_detection.stop_listening()

    def start_detect_shelf_layers(self, shelf_system_id):
        self.set_ring_light(True)
        req = RSQueryServiceRequest()
        q = {"scan":
                 {"type": "shelf",
                  "location": self.knowrob.get_perceived_frame_id(shelf_system_id),
                  "command": "start"}}
        req.query = json.dumps(q)
        self.print_with_prefix('sending: {}'.format(q))
        r = self.robosherlock_service.call(req)
        self.print_with_prefix('got: {}'.format(r))

    def rs_pose_to_geom_msgs_pose(self, pose):
        p = PoseStamped()
        p.header.frame_id = pose['frame']
        p.header.stamp = rospy.Time(pose['timestamp'] / 1000000000)
        p.pose.position.x = pose['translation'][0]
        p.pose.position.y = pose['translation'][1]
        p.pose.position.z = pose['translation'][2]
        p.pose.orientation.x = pose['rotation'][0]
        p.pose.orientation.y = pose['rotation'][1]
        p.pose.orientation.z = pose['rotation'][2]
        p.pose.orientation.w = pose['rotation'][3]
        return p

    def stop_detect_shelf_layers(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :return: list of shelf layer heights
        :rtype: list
        """
        shelf_frame = self.knowrob.get_perceived_frame_id(shelf_system_id)
        req = RSQueryServiceRequest()
        q = {'scan':
                 {'type': 'shelf',
                  'location': shelf_frame,
                  'command': 'stop'}}
        req.query = json.dumps(q)
        self.print_with_prefix('sending: {}'.format(q))
        result = self.robosherlock_service.call(req)
        self.print_with_prefix('received: {}'.format(result))
        floors = []
        for floor in result.answer:
            pose = json.loads(floor)['rs.annotation.PoseAnnotation'][0]['camera']['rs.tf.StampedPose']
            p = self.rs_pose_to_geom_msgs_pose(pose)

            # p = message_converter.convert_dictionary_to_ros_message('geometry_msgs/PoseStamped',
            #                                                         json.loads(floor)['poses'][0]['pose_stamped'])
            # TODO potential speedup, safe and reuse transform
            p = transform_pose(shelf_frame, p)
            floors.append(p.pose.position.z)
        floors = list(sorted(floors))
        # floors = [x[-1] for x in floors]
        floors = add_bottom_layer_if_not_present(floors, shelf_system_id, self.knowrob)

        return floors

    def count_product(self, facing_id):
        self.knowrob.compute_shelf_product_type(facing_id)
        self.set_ring_light(True)
        shelf_layer_id = self.knowrob.get_shelf_layer_from_facing(facing_id)
        shelf_system_id = self.knowrob.get_shelf_system_from_layer(shelf_layer_id)
        perceived_shelf_frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
        q = {'detect': {
            'facing': facing_id,
            'location': perceived_shelf_frame_id
        }}
        self.print_with_prefix('sending: {}'.format(q))
        req = RSQueryServiceRequest()
        req.query = json.dumps(q)
        rospy.sleep(0.8)
        result = self.robosherlock_service.call(req)
        self.print_with_prefix('received: {}'.format(result))
        count = 0
        predicted = ''
        expected = ''
        if len(result.answer):
            try:
                for answer in result.answer:
                    r = json.loads(answer)
                    if r['rs.annotation.Detection'][0]['source'] == 'ProductCounter':
                        count += 1
                    else:
                        predicted = r['rs.annotation.Classification'][0]['classname']
                        expected = self.add_gtin_checksum(r['rs.annotation.Detection'][0]['name'])
                # result_dict = [json.loads(x)['rs.annotation.Detection'] for x in result.answer]
                # for r in result_dict:
                #     if r[0]['source'] == 'FacingDetection':
                #         expected_gtin = self.add_gtin_checksum(r[0]['name'])
                #         detected_gtin = self.add_gtin_checksum(r[0]['name'])
                #     elif r[0]['source'] == 'ProductCounter':
                #         count += 1
                # count = json.loads(result.answer[0])['rs_refills.refills.ProductCount'][0]['product_count']
                # confidence = [x for x in result_dict if x[0]['source'] == 'FacingDetection'][0][0]['confidence']
            except:
                result.answer = [0, 0]
                rospy.logerr(result.answer)
            # self.knowrob.assert_confidence(facing_id, confidence)
        # count = max(0, len(result.answer) - 1)
        return count, predicted, expected

    def add_gtin_checksum(self, gtin):
        s = int(gtin[0])*1 + \
               int(gtin[1])*3 + \
               int(gtin[2])*1 + \
               int(gtin[3])*3 + \
               int(gtin[4])*1 + \
               int(gtin[5])*3 + \
               int(gtin[6])*1 + \
               int(gtin[7])*3 + \
               int(gtin[8])*1 + \
               int(gtin[9])*3 + \
               int(gtin[10])*1 + \
               int(gtin[11])*3
        return gtin+str(((s//10)+1) * 10 - s)


    def see(self, depth, width, height, object_type, num_detections=3):
        """
        BaleaReinigungsmilchVital:1
        DenkMitGeschirrReinigerNature:2
        DMRoteBeteSaftBio:3
        GarnierMineralUltraDry:4

        :param depth:
        :param width:
        :param height:
        :return: PoseStamped
        """
        q = {'detect': {'type': object_type}}
        self.print_with_prefix('sending: {}'.format(q))
        req = RSQueryServiceRequest()
        req.query = json.dumps(q)
        # result = self.robosherlock_service.call(req)
        pass
        rospy.sleep(0.4)
        objects = []
        expected_volume = self.volume(depth, width, height)
        while len(objects) < num_detections:
            result = self.robosherlock_service.call(req)
            answers = [json.loads(x) for x in result.answer]
            if len(answers) == 0:
                continue
            real_object = DetectedObject(answers[0])
            # real_object = min(answers, key=lambda x: abs(self.answer_volume(x) - expected_volume))
            if abs(1 - real_object.volume / expected_volume) > 0.2:  # 175: # reject if more than x% diff
                continue
            # pose.pose.position.z += real_object['boundingbox']['dimensions-3D']['height']/2.
            objects.append(real_object)
            self.print_with_prefix('found pose number {}'.format(len(objects)))
        objects = self.filter_outlier(objects)
        avg_object = self.avg_object(objects)
        # p.pose.position.z += depth/2.
        avg_object.map_T_object = transform_pose('map', avg_object.camera_T_object)
        avg_object.fix_dimensions(depth, width, height)
        return avg_object

    def avg_object(self, objects):
        avg_position = np.array([np.array([p.camera_T_object.pose.position.x,
                                           p.camera_T_object.pose.position.y,
                                           p.camera_T_object.pose.position.z]) for p in objects]).mean(axis=0)
        avg_orientation = np.array([np.array([p.camera_T_object.pose.orientation.x,
                                              p.camera_T_object.pose.orientation.y,
                                              p.camera_T_object.pose.orientation.z,
                                              p.camera_T_object.pose.orientation.w]) for p in objects]).mean(axis=0)
        avg_orientation = avg_orientation / np.linalg.norm(avg_orientation)
        avg = PoseStamped()
        avg.header.frame_id = objects[0].camera_T_object.header.frame_id
        avg.pose.position = Point(*avg_position)
        avg.pose.orientation = Quaternion(*avg_orientation)
        avg_object = DetectedObject(depth=np.average([o.depth for o in objects]),
                                    width=np.average([o.width for o in objects]),
                                    height=np.average([o.height for o in objects]),
                                    camera_T_object=avg)
        return avg_object

    def filter_outlier(self, objects, threshold=0.0007):
        positions = np.array([np.array([p.camera_T_object.pose.position.x,
                                        p.camera_T_object.pose.position.y,
                                        p.camera_T_object.pose.position.z]) for p in objects])
        avg = positions.mean(axis=0)
        return [x for i, x in enumerate(objects) if np.linalg.norm(positions[i] - avg) > threshold]

    def answer_volume(self, answer):
        return self.volume(answer['boundingbox']['dimensions-3D']['depth'],
                           answer['boundingbox']['dimensions-3D']['width'],
                           answer['boundingbox']['dimensions-3D']['height'])

    def answer_front_area(self, answer):
        return answer['boundingbox']['dimensions-3D']['depth'] * \
               answer['boundingbox']['dimensions-3D']['width']

    def volume(self, depth, width, height):
        return depth * width * height


if __name__ == '__main__':
    rospy.init_node('roboscafsdafsdf')
    robosherlock = RoboSherlock(None, check_camera=False)
    robosherlock.see(0, 0, 0)
