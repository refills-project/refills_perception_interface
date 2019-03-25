import json
import string
import traceback
import yaml
from collections import OrderedDict, defaultdict
from multiprocessing import Lock
from rospkg import RosPack

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np

from rospy_message_converter.message_converter import convert_dictionary_to_ros_message
from std_srvs.srv import Trigger, TriggerRequest
from visualization_msgs.msg import Marker

from refills_perception_interface.not_hacks import add_separator_between_barcodes, add_edge_separators, \
    merge_close_separators, merge_close_shelf_layers
from refills_perception_interface.tfwrapper import transform_pose, lookup_pose, lookup_transform
from refills_perception_interface.utils import print_with_prefix, ordered_load
from json_prolog import json_prolog
from json_prolog.json_prolog import PrologException

MAP = 'map'
SHOP = 'shop'
SHELF_FLOOR = '{}:\'ShelfLayer\''.format(SHOP)
DM_MARKET = 'dmshop'
SHELF_BOTTOM_LAYER = '{}:\'DMShelfBFloor\''.format(DM_MARKET)
SHELF_SYSTEM = '{}:\'DMShelfFrame\''.format(DM_MARKET)
SHELFH200 = '{}:\'DMShelfH200\''.format(DM_MARKET)
SHELF_T5 = '{}:\'DMShelfT5\''.format(DM_MARKET)
SHELF_T6 = '{}:\'DMShelfT6\''.format(DM_MARKET)
SHELF_T7 = '{}:\'DMShelfT7\''.format(DM_MARKET)
SHELF_W60 = '{}:\'DMShelfW60\''.format(DM_MARKET)
SHELF_W75 = '{}:\'DMShelfW75\''.format(DM_MARKET)
SHELF_W100 = '{}:\'DMShelfW100\''.format(DM_MARKET)
SHELF_W120 = '{}:\'DMShelfW120\''.format(DM_MARKET)
SHELF_H = '{}:\'DMShelfH\''.format(DM_MARKET)
SHELF_L = '{}:\'DMShelfL\''.format(DM_MARKET)

SEPARATOR = '{}:\'DMShelfSeparator4Tiles\''.format(DM_MARKET)
MOUNTING_BAR = '{}:\'DMShelfMountingBar\''.format(DM_MARKET)
BARCODE = '{}:\'DMShelfLabel\''.format(DM_MARKET)
PERCEPTION_AFFORDANCE = '{}:\'DMShelfPerceptionAffordance\''.format(DM_MARKET)

OBJECT_ACTED_ON = '\'http://knowrob.org/kb/knowrob.owl#objectActedOn\''
GOAL_LOCATION = '\'http://knowrob.org/kb/knowrob.owl#goalLocation\''
DETECTED_OBJECT = '\'http://knowrob.org/kb/knowrob.owl#detectedObject\''


MAX_SHELF_HEIGHT = 1.1

class KnowRob(object):
    prefix = 'knowrob_wrapper'

    def __init__(self):
        super(KnowRob, self).__init__()
        self.read_left_right_json()
        self.separators = {}
        self.perceived_frame_id_map = {}
        self.print_with_prefix('waiting for knowrob')
        self.prolog = json_prolog.Prolog()
        self.print_with_prefix('knowrob showed up')
        self.query_lock = Lock()
        self.reset_object_state_publisher = rospy.ServiceProxy('/object_state_publisher/update_object_positions',
                                                               Trigger)
        self.shelf_layer_from_facing = {}
        self.shelf_system_from_layer = {}

    def print_with_prefix(self, msg):
        """
        :type msg: str
        """
        print_with_prefix(msg, self.prefix)

    def once(self, q):
        r = self.all_solutions(q)
        if len(r) == 0:
            return []
        return r[0]

    def all_solutions(self, q):
        self.print_with_prefix(q)
        r = self.prolog.all_solutions(q)
        self.print_with_prefix('result: {}'.format(r))
        return r

    def pose_to_prolog(self, pose_stamped):
        """
        :type pose_stamped: PoseStamped
        :return: PoseStamped in a form the knowrob likes
        :rtype: str
        """
        return '[\'{}\', _, [{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
                                                               pose_stamped.pose.position.x,
                                                               pose_stamped.pose.position.y,
                                                               pose_stamped.pose.position.z,
                                                               pose_stamped.pose.orientation.x,
                                                               pose_stamped.pose.orientation.y,
                                                               pose_stamped.pose.orientation.z,
                                                               pose_stamped.pose.orientation.w)

    def prolog_to_pose_msg(self, query_result):
        """
        :type query_result: list
        :rtype: PoseStamped
        """
        ros_pose = PoseStamped()
        ros_pose.header.frame_id = query_result[0]
        ros_pose.pose.position = Point(*query_result[2])
        ros_pose.pose.orientation = Quaternion(*query_result[3])
        return ros_pose

    def read_left_right_json(self):
        try:
            self.path_to_json = rospy.get_param('~path_to_json')
            self.left_right_dict = OrderedDict()
            with open(self.path_to_json, 'r') as f:
                self.left_right_dict = ordered_load(f, yaml.SafeLoader)
            prev_id = None
            for i, shelf_system_id in enumerate(self.left_right_dict):
                if i > 0 and prev_id != self.left_right_dict[shelf_system_id]['starting-point']:
                    raise TypeError('starting point doesn\'t match the prev entry at {}'.format(shelf_system_id))
                prev_id = shelf_system_id
                via_points = self.left_right_dict[shelf_system_id]['via-points']
                for i in range(len(via_points)):
                    via_points[i] = convert_dictionary_to_ros_message("geometry_msgs/PoseStamped", via_points[i])
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn('failed to load left right json')

    def is_left(self, shelf_system_id):
        return self.left_right_dict[shelf_system_id]['side'] == 'left'

    def is_right(self, shelf_system_id):
        return self.left_right_dict[shelf_system_id]['side'] == 'right'

    def get_shelf_system_ids(self, filter_with_left_right_dict=True):
        """
        :return: list of str
        :rtype: list
        """
        all_ids = set(self.get_all_individuals_of(SHELF_SYSTEM))
        if filter_with_left_right_dict:
            return [x for x in self.left_right_dict.keys() if x in all_ids]
        else:
            return all_ids

    def get_shelf_pose(self, shelf_system_id):
        return lookup_pose("map", self.get_object_frame_id(shelf_system_id))

    def get_num_of_tiles(self, shelf_system_id):
        if self.is_5tile_system(shelf_system_id):
            return 5
        elif self.is_6tile_system(shelf_system_id):
            return 6
        elif self.is_7tile_system(shelf_system_id):
            return 7
        else:
            raise Exception('Could not identify number of tiles for shelf {}.'.format(shelf_system_id))

    def is_5tile_system(self, shelf_system_id):
        q = 'rdfs_individual_of(\'{}\', {})'.format(shelf_system_id, SHELF_T5)
        return self.once(q) == {}

    def is_heavy_system(self, shelf_system_id):
        q = 'rdfs_individual_of(\'{}\', {})'.format(shelf_system_id, SHELF_H)
        return self.once(q) == {}

    def is_6tile_system(self, shelf_system_id):
        q = 'rdfs_individual_of(\'{}\', {})'.format(shelf_system_id, SHELF_T6)
        return self.once(q) == {}

    def is_7tile_system(self, shelf_system_id):
        q = 'rdfs_individual_of(\'{}\', {})'.format(shelf_system_id, SHELF_T7)
        return self.once(q) == {}

    def get_bottom_layer_type(self, shelf_system_id):
        q = 'shelf_bottom_floor_type(\'{}\', LayerType).'.format(shelf_system_id)
        return self.once(q)['LayerType']

    def get_shelf_layer_type(self, shelf_system_id):
        q = 'shelf_floor_type(\'{}\', LayerType).'.format(shelf_system_id)
        return self.once(q)['LayerType']

    def get_shelf_layer_from_system(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :return: returns dict mapping floor id to pose ordered from lowest to highest
        :rtype: dict
        """
        q = 'rdf_has(\'{}\', knowrob:properPhysicalParts, Floor), ' \
            'rdfs_individual_of(Floor, {}), ' \
            'object_perception_affordance_frame_name(Floor, Frame).'.format(shelf_system_id, SHELF_FLOOR)

        solutions = self.all_solutions(q)
        floors = []
        shelf_frame_id = self.get_perceived_frame_id(shelf_system_id)
        for solution in solutions:
            floor_id = solution['Floor'].replace('\'', '')
            floor_pose = lookup_pose(shelf_frame_id, solution['Frame'].replace('\'', ''))
            floors.append((floor_id, floor_pose))
        floors = list(sorted(floors, key=lambda x: x[1].pose.position.z))
        floors = [x for x in floors if x[1].pose.position.z < MAX_SHELF_HEIGHT]
        self.floors = OrderedDict(floors)
        return self.floors

    def get_facing_ids_from_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return:
        :rtype: OrderedDict
        """
        shelf_system_id = self.get_shelf_system_from_layer(shelf_layer_id)
        q = 'findall([F, P], (shelf_facing(\'{}\', F),belief_at(F, P)), Fs).'.format(shelf_layer_id)
        solutions = self.all_solutions(q)[0]
        facings = []
        for facing_id, pose in solutions['Fs']:
            facing_pose = self.prolog_to_pose_msg(pose)
            facing_pose = transform_pose(self.get_perceived_frame_id(shelf_layer_id), facing_pose)
            facings.append((facing_id, facing_pose))
        is_left = 1 if self.is_left(shelf_system_id) else -1
        facings = list(sorted(facings, key=lambda x: x[1].pose.position.x * is_left))
        return OrderedDict(facings)

    def shelf_system_exists(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: bool
        """
        return shelf_system_id in self.get_shelf_system_ids()

    def shelf_layer_exists(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: bool
        """
        q = 'shelf_layer_frame(\'{}\', _).'.format(shelf_layer_id)
        return self.once(q) == {}

    def facing_exists(self, facing_id):
        """
        :type facing_id: str
        :rtype: bool
        """
        q = 'shelf_facing(L, \'{}\').'.format(facing_id)
        return len(self.all_solutions(q)) != 0

    def belief_at_update(self, id, pose):
        """
        :type id: str
        :type pose: PoseStamped
        """
        q = 'belief_at_update(\'{}\', {})'.format(id, self.pose_to_prolog(pose))
        return self.once(q)

    # def get_objects(self, object_type):
    #     """
    #     Ask knowrob for a specific type of objects
    #     :type object_type: str
    #     :return: all objects of the given type
    #     :rtype: dict
    #     """
    #     objects = OrderedDict()
    #     q = 'rdfs_individual_of(R, {}).'.format(object_type)
    #     solutions = self.all_solutions(q)
    #     for solution in solutions:
    #         object_id = solution['R'].replace('\'', '')
    #         pose_q = 'belief_at(\'{}\', R).'.format(object_id)
    #         believed_pose = self.once(pose_q)['R']
    #         ros_pose = PoseStamped()
    #         ros_pose.header.frame_id = believed_pose[0]
    #         ros_pose.pose.position = Point(*believed_pose[2])
    #         ros_pose.pose.orientation = Quaternion(*believed_pose[3])
    #         objects[str(object_id)] = ros_pose
    #     return objects

    def get_all_individuals_of(self, object_type):
        q = ' findall(R, rdfs_individual_of(R, {}), Rs).'.format(object_type)
        solutions = self.once(q)['Rs']
        return [self.remove_quotes(solution) for solution in solutions]

    def remove_quotes(self, s):
        return s.replace('\'', '')

    # def belief_at(self, object_id):
    #     pose_q = 'belief_at(\'{}\', R).'.format(object_id)
    #     believed_pose = self.once(pose_q)['R']
    #     ros_pose = PoseStamped()
    #     ros_pose.header.frame_id = believed_pose[0]
    #     ros_pose.pose.position = Point(*believed_pose[2])
    #     ros_pose.pose.orientation = Quaternion(*believed_pose[3])
    #     return ros_pose

    def get_perceived_frame_id(self, object_id):
        """
        :type object_id: str
        :return: the frame_id of an object according to the specifications in our wiki.
        :rtype: str
        """
        if object_id not in self.perceived_frame_id_map:
            q = 'object_perception_affordance_frame_name(\'{}\', F)'.format(object_id)
            self.perceived_frame_id_map[object_id] = self.once(q)['F'].replace('\'', '')
        return self.perceived_frame_id_map[object_id]

    def get_object_frame_id(self, object_id):
        """
        :type object_id: str
        :return: frame_id of the center of mesh.
        :rtype: str
        """
        q = 'object_frame_name(\'{}\', R).'.format(object_id)
        return self.once(q)['R'].replace('\'', '')

    # floor
    def add_shelf_layers(self, shelf_system_id, shelf_layer_heights):
        """
        :param shelf_system_id: layers will be attached to this shelf system.
        :type shelf_system_id: str
        :param shelf_layer_heights: heights of the detects layers, list of floats
        :type shelf_layer_heights: list
        :return: TODO
        :rtype: bool
        """
        shelf_layer_heights = merge_close_shelf_layers(shelf_layer_heights)
        for i, height in enumerate(sorted(shelf_layer_heights)):
            if i == 0:
                layer_type = self.get_bottom_layer_type(shelf_system_id)
            else:
                layer_type = self.get_shelf_layer_type(shelf_system_id)
            q = 'belief_shelf_part_at(\'{}\', {}, {}, R)'.format(shelf_system_id, layer_type, height)
            self.once(q)
        return True

    def update_shelf_layer_position(self, shelf_layer_id, separators):
        """
        :type shelf_layer_id: str
        :type separators: list of PoseStamped, positions of separators
        """
        #TODO possibile speed but, reuse transform
        separator_zs = [transform_pose(self.get_perceived_frame_id(shelf_layer_id), p).pose.position.z for p in separators]
        if len(separator_zs) > 0:
            new_floor_height = np.mean(separator_zs)
            current_floor_pose = lookup_pose(MAP, self.get_object_frame_id(shelf_layer_id))
            current_floor_pose.pose.position.z += new_floor_height
            q = 'belief_at_update(\'{}\', {})'.format(shelf_layer_id, self.pose_to_prolog(current_floor_pose))
            self.once(q)

    def add_separators(self, shelf_layer_id, separators):
        """
        :param shelf_layer_id: separators will be attached to this shelf layer.
        :type shelf_layer_id: str
        :param separators: list of PoseStamped, positions of separators
        :return:
        """
        # TODO check success
        for p in separators:
            q = 'belief_shelf_part_at(\'{}\', {}, {}, _)'.format(shelf_layer_id, SEPARATOR, p.pose.position.x)
            try:
                self.once(q)
            except Exception as e:
                traceback.print_exc()
                return False
        return True

    def add_barcodes(self, shelf_layer_id, barcodes):
        """
        :param shelf_layer_id: barcodes will be attached to this shelf layer
        :type shelf_layer_id: str
        :param barcodes: dict mapping barcode to PoseStamped. make sure it relative to shelf layer, everything but x ignored
        :type barcodes: dict
        """
        # TODO check success
        for barcode, p in barcodes.items():
            if not self.does_DAN_exist(barcode):
                q = 'create_article_number(dan(\'{}\'),AN), ' \
                    'create_article_type(AN,[{},{},{}],ProductType).'.format(barcode, 0.4, 0.015, 0.1)
                r = self.once(q)
            q = 'belief_shelf_barcode_at(\'{}\', {}, dan(\'{}\'), {}, _).'.format(shelf_layer_id, BARCODE,
                                                                                  barcode, p.pose.position.x)
            self.once(q)

    def create_unknown_barcodes(self, barcodes):
        for barcode, p in barcodes.items():
            if not self.does_DAN_exist(barcode):
                q = 'create_article_number(dan(\'{}\'),AN), ' \
                    'create_article_type(AN,[{},{},{}],ProductType).'.format(barcode, 0.4, 0.015, 0.1)
                r = self.once(q)

    def add_separators_and_barcodes(self, shelf_layer_id, separators, barcodes):
        shelf_layer_width = self.get_shelf_layer_width(shelf_layer_id)
        separators_xs = [p.pose.position.x / shelf_layer_width for p in separators]
        barcodes = [(p.pose.position.x/shelf_layer_width, barcode) for barcode, p in barcodes.items()]

        # definitely no hacks here
        separators_xs, barcodes = add_separator_between_barcodes(separators_xs, barcodes)
        separators_xs = add_edge_separators(separators_xs)
        separators_xs = merge_close_separators(separators_xs)

        q = 'bulk_insert_floor(\'{}\', separators({}), labels({}))'.format(shelf_layer_id, separators_xs, barcodes)
        self.once(q)

    def assert_confidence(self, facing_id, confidence):
        q = 'rdf_assert(\'{}\', knowrob:confidence, literal(type(xsd:double, \'{}\')), belief_state).'.format(facing_id, confidence)
        self.once(q)

    def does_DAN_exist(self, dan):
        q = 'article_number_of_dan(\'{}\', _)'.format(dan)
        return self.once(q) == {}

    def get_all_product_dan(self):
        """
        :return: list of str
        :rtype: list
        """
        q = 'findall(DAN,rdf_has_prolog(AN,shop:dan,DAN), DANS)'
        dans = self.once(q)['DANS']
        return dans

    def add_objects(self, facing_id, number):
        """
        Adds objects to the facing whose type is according to the barcode.
        :type facing_id: str
        :type number: int
        """
        for i in range(number):
            q = 'product_spawn_front_to_back(\'{}\', ObjId)'.format(facing_id)
            self.once(q)

    def save_beliefstate(self, path=None):
        """
        :type path: str
        """
        if path is None:
            path = '{}/data/beliefstate.owl'.format(RosPack().get_path('refills_first_review'))
        q = 'belief_export(\'{}\')'.format(path)
        self.once(q)

    def get_shelf_layer_width(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', D, W, H).'.format(shelf_layer_id)
        solution = self.once(q)
        if solution:
            width = solution['W']
            return width
        else:
            raise Exception('width not defined for {}'.format(shelf_layer_id))

    def get_shelf_system_width(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', D, W, H).'.format(shelf_system_id)
        solution = self.once(q)
        width = solution['W']
        return width

    def get_shelf_system_height(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', D, W, H).'.format(shelf_system_id)
        solution = self.once(q)
        height = solution['H']
        return height

    def get_shelf_system_from_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: str
        """
        if shelf_layer_id not in self.shelf_system_from_layer:
            q = 'shelf_layer_frame(\'{}\', Frame).'.format(shelf_layer_id)
            shelf_system_id = self.once(q)['Frame'][1:-1]
            self.shelf_system_from_layer[shelf_layer_id] = shelf_system_id
        return self.shelf_system_from_layer[shelf_layer_id]

    def get_shelf_layer_from_facing(self, facing_id):
        """
        :type facing_id: str
        :rtype: str
        """
        if facing_id not in self.shelf_layer_from_facing:
            q = 'shelf_facing(Layer, \'{}\').'.format(facing_id)
            layer_id =  self.once(q)['Layer'][1:-1]
            self.shelf_layer_from_facing[facing_id] = layer_id
        return self.shelf_layer_from_facing[facing_id]

    def get_shelf_layer_above(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        q = 'shelf_layer_above(\'{}\', Above).'.format(shelf_layer_id)
        solution = self.once(q)
        if isinstance(solution, dict):
            return solution['Above'][1:-1]

    def is_top_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        return self.get_shelf_layer_above(shelf_layer_id) is None

    def is_bottom_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        q = 'rdfs_individual_of(\'{}\', {})'.format(shelf_layer_id, SHELF_BOTTOM_LAYER)
        return self.once(q) != []

    def clear_beliefstate(self, initial_beliefstate=None):
        """
        :rtype: bool
        """
        #put path of owl here
        if initial_beliefstate is None:
            initial_beliefstate = self.initial_beliefstate
        q = 'belief_forget, retractall(owl_parser:owl_file_loaded(\'{}\'))'.format(initial_beliefstate)
        result = self.once(q) != []
        self.reset_object_state_publisher.call(TriggerRequest())
        return result

    def reset_beliefstate(self, inital_beliefstate=None):
        """
        :rtype: bool
        """
        return self.clear_beliefstate(inital_beliefstate) and self.load_initial_beliefstate()

    def load_initial_beliefstate(self):
        self.initial_beliefstate = rospy.get_param('~initial_beliefstate')
        if self.load_owl(self.initial_beliefstate):
            print_with_prefix('loaded initial beliefstate {}'.format(self.initial_beliefstate), self.prefix)
            self.reset_object_state_publisher.call(TriggerRequest())
            return True
        else:
            print_with_prefix('error loading initial beliefstate {}'.format(self.initial_beliefstate), self.prefix)
            return False

    def load_owl(self, path):
        """
        :type path: str
        :rtype: bool
        """
        q = 'belief_parse(\'{}\')'.format(path)
        return self.once(q) != []


if __name__ == u'__main__':
    rospy.init_node('perception_interface')
    kb = KnowRob()
    kb.once('1=0.')
