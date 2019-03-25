from __future__ import print_function, division

import json
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Point
from iai_ringlight.srv import iai_ringlight_in, iai_ringlight_inRequest
from robosherlock_msgs.srv import RSQueryService, RSQueryServiceRequest
from rospy import ROSException
from rospy_message_converter import message_converter

from refills_perception_interface.barcode_detection import BarcodeDetector
from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.not_hacks import add_bottom_layer_if_not_present
from refills_perception_interface.separator_detection import SeparatorClustering
from refills_perception_interface.tfwrapper import transform_pose
from refills_perception_interface.utils import print_with_prefix, error_with_refix

MAP = 'map'


class FakeRoboSherlock(object):
    prefix = 'robosherlock wrapper'

    def __init__(self, knowrob, num_of_facings=3):
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
        self.current_shelf_layer_id = floor_id

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
            separator.pose.position = Point(x, 0.01*i, 0.01)
            separator.pose.orientation.w = 1
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
        for i in range(self.number_of_facings):
            barcode = PoseStamped()
            barcode.header.frame_id = self.knowrob.get_perceived_frame_id(self.current_shelf_layer_id)
            x = ((i + .5) / (self.number_of_facings))*width
            barcode.pose.position = Point(x, 0, 0)
            barcode.pose.orientation.w = 1
            try:
                if np.random.choice([True, False]):
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
        self.knowrob.assert_confidence(facing_id, 0.88)
        return 1

    def start_detect_shelf_layers(self, shelf_system_id):
        """
        :type shelf_system_id: str
        """
        pass

    def stop_detect_shelf_layers(self, shelf_system_id):
        """
        :return: list of shelf layer heights
        :rtype: list
        """
        # shelf_system_height = self.knowrob.get_shelf_system_height(shelf_system_id)
        shelf_system_height = 1
        detected_shelf_layers = (np.random.rand(2)*(shelf_system_height-0.2)+0.2).tolist()
        return add_bottom_layer_if_not_present(detected_shelf_layers, shelf_system_id, self.knowrob)


class RoboSherlock(FakeRoboSherlock):
    def __init__(self, knowrob):
        super(RoboSherlock, self).__init__(knowrob)
        self.knowrob = knowrob  # type: KnowRob
        # TODO camera topics as ros param
        self.rgb_topic = rospy.get_param('~rgb_topic')
        self.wait_for_rgb_camera()
        self.depth_topic = rospy.get_param('~realsense_topic')
        self.wait_for_realsense()
        self.separator_detection = SeparatorClustering(knowrob)
        self.barcode_detection = BarcodeDetector(knowrob)

        self.robosherlock_srv_name = rospy.get_param('~robosherlock_srv_name', '/RoboSherlock/query')

        self.ring_light_srv = rospy.ServiceProxy('iai_ringlight_controller', iai_ringlight_in)

        self.wait_for_robosherlock()

    def set_ring_light(self, value=True):
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
            rospy.wait_for_service(self.robosherlock_srv_name, timeout=5)
        except ROSException as e:
            self.error_with_prefix('robosherlock unavailable ({})'.format(self.robosherlock_srv_name))
            raise e
        self.robosherlock_service = rospy.ServiceProxy('/RoboSherlock/query', RSQueryService)
        self.print_with_prefix('connected to RoboSherlock')

    def wait_for_rgb_camera(self):
        self.print_with_prefix('waiting for rgb camera')
        try:
            rospy.wait_for_message(self.rgb_topic, rospy.AnyMsg, timeout=5)
        except ROSException as e:
            self.error_with_prefix('rgb camera unavailable ({})'.format(self.rgb_topic))
            raise e
        self.print_with_prefix('rgb camera found')

    def wait_for_realsense(self):
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

    def rs_pose_to_geom_msgs_pose(self,  pose):
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
        rospy.sleep(0.4)
        result = self.robosherlock_service.call(req)
        self.print_with_prefix('received: {}'.format(result))
        count = 0
        if len(result.answer):
            try:
                # result_dict = [json.loads(x)['detection'] for x in result.answer]
                count = json.loads(result.answer[0])['rs_refills.refills.ProductCount'][0]['product_count']
                # confidence = [x for x in result_dict if x['source'] == 'FacingDetection'][0]['confidence']
            except:
                confidence = 0.01
                result.answer = [0,0]
                rospy.logerr(result.answer)
            self.knowrob.assert_confidence(facing_id, confidence)
        # count = max(0,len(result.answer) - 1)
        return count
