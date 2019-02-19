#!/usr/bin/env python
from Queue import Queue
from random import seed, choice, randint

import rospy
from actionlib import SimpleActionServer
from refills_msgs.msg import DetectShelfLayersAction, DetectFacingsAction, CountProductsAction, DetectShelfLayersGoal, \
    DetectShelfLayersResult, DetectFacingsActionResult, DetectFacingsActionGoal, DetectFacingsResult, \
    CountProductsResult, CountProductsGoal, DetectFacingsGoal, FullBodyPosture, JointPosition
from refills_msgs.srv import QueryShelfSystems, QueryShelfSystemsRequest, QueryShelfSystemsResponse, QueryShelfLayers, \
    QueryFacings, FinishPerception, FinishPerceptionRequest, FinishPerceptionResponse, QueryFacingsRequest, \
    QueryFacingsResponse, \
    QueryShelfLayersRequest, QueryShelfLayersResponse, QueryDetectShelfLayersPath, QueryDetectFacingsPath, \
    QueryCountProductsPosture, QueryDetectShelfLayersPathRequest, QueryDetectShelfLayersPathResponse, \
    QueryDetectFacingsPathResponse, QueryDetectFacingsPathRequest, QueryCountProductsPostureResponse, \
    QueryCountProductsPostureRequest
import string
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import numpy as np

from std_srvs.srv import TriggerResponse, Trigger
from tf.transformations import quaternion_about_axis

seed(109117104)
NUM_SHELVES = 3
NUM_LAYER = 3
NUM_FACINGS = 3

ONE_YEAR = 365 * 24 * 60 * 60

joint_names = ['Torso_lin1', 'Torso_lin2', 'Torso_rot_1']


def rnd_hash(length=6):
    return ''.join(choice([x for x in 'abcdef' + string.digits]) for _ in range(length))


# TODO only fill 'knowrob' after detection
# TODO some example of how to use the interface with actual msgs

class DummyInterface(object):
    READY = -1

    KILL_DETECT_SHELF_LAYER = 0
    KILL_DETECT_FACINGS = 1
    KILL_COUNT_PRODUCTS = 2

    BUSY_DETECT_SHELF_LAYER = 3
    BUSY_DETECT_FACINGS = 4
    BUSY_COUNT_PRODUCTS = 5

    FINISH = 6
    ABORT = 7

    def __init__(self):
        self.query_shelf_systems_srv = rospy.Service('~query_shelf_systems', QueryShelfSystems,
                                                     self.query_shelf_systems_cb)
        self.query_shelf_layers_srv = rospy.Service('~query_shelf_layers', QueryShelfLayers, self.query_shelf_layers_cb)
        self.query_facings_srv = rospy.Service('~query_facings', QueryFacings, self.query_facings_cb)
        self.query_shelf_layer_detection_path_srv = rospy.Service('~query_detect_shelf_layers_path',
                                                                  QueryDetectShelfLayersPath,
                                                                  self.query_detect_shelf_layers_path_cb)
        self.query_facing_detection_path_srv = rospy.Service('~query_detect_facings_path', QueryDetectFacingsPath,
                                                             self.query_detect_facings_path_cb)
        self.query_product_counting_path_srv = rospy.Service('~query_count_products_posture',
                                                             QueryCountProductsPosture,
                                                             self.query_count_products_posture_cb)
        self.finish_perception_srv = rospy.Service('~finish_perception', FinishPerception, self.finish_perception_cb)
        self.query_reset_beliefstate_srv = rospy.Service('~reset_beliefstate', Trigger, lambda x:TriggerResponse())

        self.detect_shelf_layer_as = SimpleActionServer('~detect_shelf_layers', DetectShelfLayersAction,
                                                        execute_cb=self.detect_shelf_layers_cb, auto_start=False)
        self.detect_shelf_layer_as.register_preempt_callback(self.cancel_cb)
        self.detect_facings_as = SimpleActionServer('~detect_facings', DetectFacingsAction,
                                                    execute_cb=self.detect_facings_cb, auto_start=False)
        self.detect_facings_as.register_preempt_callback(self.cancel_cb)
        self.count_products_as = SimpleActionServer('~count_products', CountProductsAction,
                                                    execute_cb=self.count_products_cb, auto_start=False)
        self.count_products_as.register_preempt_callback(self.cancel_cb)

        self.state_lock = Queue(1)

        self.detect_shelf_layer_as.start()
        self.detect_facings_as.start()
        self.count_products_as.start()
        self.set_ready(prefix='init')
        self.set_names()

    def set_names(self):
        self.shelf_ids = ['shelf_535dc8']
        self.layer_ids = ['layer_faa57d']
        self.facing_ids = ['facing_92c6c2', 'facing_77bce2', 'facing_c4a62e']
        self.shelf_to_layer = {'shelf_535dc8': ['layer_faa57d']}
        self.layer_to_facing = {'layer_faa57d': ['facing_92c6c2', 'facing_77bce2', 'facing_c4a62e']}
        self.product_counts = {}

        # for shelf_id in self.shelf_ids:
        #     self.shelf_to_layer[shelf_id] = ['layer_{}'.format(rnd_hash()) for x in range(NUM_LAYER)]
        #     self.layer_ids.extend(self.shelf_to_layer[shelf_id])
        # for layer_id in self.layer_ids:
        #     self.layer_to_facing[layer_id] = ['facing_{}'.format(rnd_hash()) for x in range(NUM_FACINGS)]
        #     self.facing_ids.extend(self.layer_to_facing[layer_id])
        for facing_id in self.facing_ids:
            self.product_counts[facing_id] = randint(0, 5)

    def query_shelf_systems_cb(self, data):
        """
        :type data: QueryShelfSystemsRequest
        :rtype: QueryShelfSystemsResponse
        """
        prefix = 'query_shelf_systems'
        self.print_with_prefix('called', prefix)
        r = QueryShelfSystemsResponse()
        r.ids = self.shelf_ids
        return r

    def query_shelf_layers_cb(self, data):
        """
        :type data: QueryShelfLayersRequest
        :rtype: QueryShelfLayersResponse
        """
        prefix = 'query_shelf_layers'
        self.print_with_prefix('called', prefix)
        r = QueryShelfLayersResponse()
        if data.id in self.shelf_ids:
            r.error = QueryShelfLayersResponse.SUCCESS
            r.ids = self.shelf_to_layer[data.id]
        else:
            self.print_with_prefix('invalid id', prefix)
            r.error = QueryShelfLayersResponse.INVALID_ID
        return r

    def query_facings_cb(self, data):
        """
        :type data: QueryFacingsRequest
        :rtype: QueryFacingsResponse
        """
        prefix = 'query_facings'
        self.print_with_prefix('called', prefix)
        r = QueryFacingsResponse()
        if data.id in self.layer_ids:
            r.error = QueryFacingsResponse.SUCCESS
            r.ids = self.layer_to_facing[data.id]
        else:
            self.print_with_prefix('invalid id', prefix)
            r.error = QueryFacingsResponse.INVALID_ID
        return r

    def query_detect_shelf_layers_path_cb(self, data):
        """
        :type data: QueryDetectShelfLayersPathRequest
        :rtype: QueryDetectShelfLayersPathResponse
        """
        self.print_with_prefix('called', 'query_detect_shelf_layers_path')
        r = QueryDetectShelfLayersPathResponse()

        if data.id in self.shelf_ids:
            r.error = QueryDetectShelfLayersPathResponse.SUCCESS
            base_pose = PoseStamped()
            base_pose.header.frame_id = '/map'
            base_pose.pose.position = Point(rospy.get_param('shelf_detection_path/position/x'),
                                            rospy.get_param('shelf_detection_path/position/y'),
                                            rospy.get_param('shelf_detection_path/position/z'))
            base_pose.pose.orientation = Quaternion(rospy.get_param('shelf_detection_path/orientation/x'),
                                                    rospy.get_param('shelf_detection_path/orientation/y'),
                                                    rospy.get_param('shelf_detection_path/orientation/z'),
                                                    rospy.get_param('shelf_detection_path/orientation/w'))

            for k in range(5):
                posture = FullBodyPosture()
                posture.base_pos = base_pose
                posture.joints.append(JointPosition('Torso_lin1',
                                                    rospy.get_param('shelf_detection_path/Torso_lin1')))
                posture.joints.append(JointPosition('Torso_lin2',
                                                    rospy.get_param('shelf_detection_path/Torso_lin2/gain')*k))
                posture.joints.append(JointPosition('Torso_rot_1',
                                                    rospy.get_param('shelf_detection_path/Torso_rot_1')))
                r.path.postures.append(posture)
        else:
            r.error = QueryDetectShelfLayersPathResponse.INVALID_ID
        return r

    def query_detect_facings_path_cb(self, data):
        """
        :type data: QueryDetectFacingsPathRequest
        :rtype: QueryDetectFacingsPathResponse
        """
        self.print_with_prefix('called', 'query_detect_facings_path')
        r = QueryDetectFacingsPathResponse()
        if data.id in self.layer_ids:
            r.error = QueryDetectFacingsPathResponse.SUCCESS
            # Start
            posture = FullBodyPosture()
            posture.base_pos.header.frame_id = '/map'
            posture.base_pos.pose.position = Point(rospy.get_param('detect_facing_path/start/position/x'),
                                                   rospy.get_param('detect_facing_path/start/position/y'),
                                                   rospy.get_param('detect_facing_path/start/position/z'))
            posture.base_pos.pose.orientation = Quaternion(rospy.get_param('detect_facing_path/start/orientation/x'),
                                                           rospy.get_param('detect_facing_path/start/orientation/y'),
                                                           rospy.get_param('detect_facing_path/start/orientation/z'),
                                                           rospy.get_param('detect_facing_path/start/orientation/w'))
            posture.joints.append(JointPosition('Torso_lin1',
                                                rospy.get_param('detect_facing_path/start/Torso_lin1')))
            posture.joints.append(JointPosition('Torso_lin2',
                                                rospy.get_param('detect_facing_path/start/Torso_lin2')))
            posture.joints.append(JointPosition('Torso_rot_1',
                                                rospy.get_param('detect_facing_path/start/Torso_rot_1')))
            r.path.postures.append(posture)

            # End
            posture = FullBodyPosture()
            posture.base_pos.header.frame_id = '/map'
            posture.base_pos.pose.position = Point(rospy.get_param('detect_facing_path/end/position/x'),
                                                   rospy.get_param('detect_facing_path/end/position/y'),
                                                   rospy.get_param('detect_facing_path/end/position/z'))
            posture.base_pos.pose.orientation = Quaternion(rospy.get_param('detect_facing_path/end/orientation/x'),
                                                           rospy.get_param('detect_facing_path/end/orientation/y'),
                                                           rospy.get_param('detect_facing_path/end/orientation/z'),
                                                           rospy.get_param('detect_facing_path/end/orientation/w'))
            posture.joints.append(JointPosition('Torso_lin1',
                                                rospy.get_param('detect_facing_path/end/Torso_lin1')))
            posture.joints.append(JointPosition('Torso_lin2',
                                                rospy.get_param('detect_facing_path/end/Torso_lin2')))
            posture.joints.append(JointPosition('Torso_rot_1',
                                                rospy.get_param('detect_facing_path/end/Torso_rot_1')))
            r.path.postures.append(posture)

        else:
            r.error = QueryDetectFacingsPathResponse.INVALID_ID
        return r

    def query_count_products_posture_cb(self, data):
        """
        :type data: QueryCountProductsPostureRequest
        :rtype: QueryCountProductsPostureResponse
        """
        self.print_with_prefix('called', 'query_count_products_posture')
        r = QueryCountProductsPostureResponse()
        if data.id in self.facing_ids:
            r.error = QueryCountProductsPostureResponse.SUCCESS
            if data.id == 'facing_92c6c2':
                r.posture = FullBodyPosture()
                r.posture.base_pos.header.frame_id = '/map'
                r.posture.base_pos.pose.position = Point(rospy.get_param('count_products_posture/facing_92c6c2/position/x'),
                                                         rospy.get_param('count_products_posture/facing_92c6c2/position/y'),
                                                         rospy.get_param('count_products_posture/facing_92c6c2/position/z'))
                r.posture.base_pos.pose.orientation = Quaternion(rospy.get_param('count_products_posture/facing_92c6c2/orientation/x'),
                                                                 rospy.get_param('count_products_posture/facing_92c6c2/orientation/y'),
                                                                 rospy.get_param('count_products_posture/facing_92c6c2/orientation/z'),
                                                                 rospy.get_param('count_products_posture/facing_92c6c2/orientation/w'))
                r.posture.joints.append(JointPosition('Torso_lin1',
                                                    rospy.get_param('count_products_posture/facing_92c6c2/Torso_lin1')))
                r.posture.joints.append(JointPosition('Torso_lin2',
                                                    rospy.get_param('count_products_posture/facing_92c6c2/Torso_lin2')))
                r.posture.joints.append(JointPosition('Torso_rot_1',
                                                    rospy.get_param('count_products_posture/facing_92c6c2/Torso_rot_1')))

            elif data.id == 'facing_77bce2':
                r.posture = FullBodyPosture()
                r.posture.base_pos.header.frame_id = '/map'
                r.posture.base_pos.pose.position = Point(rospy.get_param('count_products_posture/facing_77bce2/position/x'),
                                                         rospy.get_param('count_products_posture/facing_77bce2/position/y'),
                                                         rospy.get_param('count_products_posture/facing_77bce2/position/z'))
                r.posture.base_pos.pose.orientation = Quaternion(rospy.get_param('count_products_posture/facing_77bce2/orientation/x'),
                                                                 rospy.get_param('count_products_posture/facing_77bce2/orientation/y'),
                                                                 rospy.get_param('count_products_posture/facing_77bce2/orientation/z'),
                                                                 rospy.get_param('count_products_posture/facing_77bce2/orientation/w'))
                r.posture.joints.append(JointPosition('Torso_lin1',
                                                    rospy.get_param('count_products_posture/facing_77bce2/Torso_lin1')))
                r.posture.joints.append(JointPosition('Torso_lin2',
                                                    rospy.get_param('count_products_posture/facing_77bce2/Torso_lin2')))
                r.posture.joints.append(JointPosition('Torso_rot_1',
                                                    rospy.get_param('count_products_posture/facing_77bce2/Torso_rot_1')))

            elif data.id == 'facing_c4a62e':
                r.posture = FullBodyPosture()
                r.posture.base_pos.header.frame_id = '/map'
                r.posture.base_pos.pose.position = Point(rospy.get_param('count_products_posture/facing_c4a62e/position/x'),
                                                         rospy.get_param('count_products_posture/facing_c4a62e/position/y'),
                                                         rospy.get_param('count_products_posture/facing_c4a62e/position/z'))
                r.posture.base_pos.pose.orientation = Quaternion(rospy.get_param('count_products_posture/facing_c4a62e/orientation/x'),
                                                                 rospy.get_param('count_products_posture/facing_c4a62e/orientation/y'),
                                                                 rospy.get_param('count_products_posture/facing_c4a62e/orientation/z'),
                                                                 rospy.get_param('count_products_posture/facing_c4a62e/orientation/w'))
                r.posture.joints.append(JointPosition('Torso_lin1',
                                                    rospy.get_param('count_products_posture/facing_c4a62e/Torso_lin1')))
                r.posture.joints.append(JointPosition('Torso_lin2',
                                                    rospy.get_param('count_products_posture/facing_c4a62e/Torso_lin2')))
                r.posture.joints.append(JointPosition('Torso_rot_1',
                                                    rospy.get_param('count_products_posture/facing_c4a62e/Torso_rot_1')))

        else:
            r.error = QueryCountProductsPostureResponse.INVALID_ID
        return r

    def finish_perception_cb(self, data):
        """
        :type data: FinishPerceptionRequest
        :rtype: FinishPerceptionResponse
        """
        prefix = 'finish_perception'
        self.print_with_prefix('service called', prefix)
        state = self.get_state(prefix)
        if self.is_ready(state) or self.is_abort(state) or self.is_finish(state):
            self.put_state(state, prefix)
            self.warn_with_prefix('server idle', prefix)
            return FinishPerceptionResponse(error=FinishPerceptionResponse.NO_RUNNING_JOB,
                                            error_msg='no running job')
        elif self.is_busy(state):
            self.set_finish(prefix)
            self.print_with_prefix('finishing {}'.format(self.state_code_to_name(state)), prefix)
            return FinishPerceptionResponse(error=FinishPerceptionResponse.SUCCESS)
        else:
            self.put_state(state, prefix)
            self.print_with_prefix('called in unexpected state {}'.format(state), prefix)

    def detect_shelf_layers_cb(self, data):
        """
        :type data: DetectShelfLayersGoal
        """
        result_type = DetectShelfLayersResult
        action_server = self.detect_shelf_layer_as
        prefix = 'detect_shelf_layers'

        self.print_with_prefix('called', prefix)
        r = result_type()
        state = self.wait_for(
            [self.READY, self.KILL_DETECT_SHELF_LAYER, self.KILL_DETECT_FACINGS, self.KILL_COUNT_PRODUCTS,
             self.BUSY_DETECT_SHELF_LAYER, self.BUSY_COUNT_PRODUCTS, self.BUSY_DETECT_FACINGS], prefix)
        if state in [self.KILL_DETECT_FACINGS, self.READY, self.KILL_COUNT_PRODUCTS]:
            if data.id in self.shelf_ids:
                self.print_with_prefix('started', prefix)
                self.put_state(self.BUSY_DETECT_SHELF_LAYER, prefix)
                state = self.wait_for([self.ABORT, self.FINISH], prefix)
                if self.is_abort(state):
                    r.error = 3
                    if action_server.new_goal:
                        self.set_ready(self.KILL_DETECT_SHELF_LAYER, prefix)
                        self.warn_with_prefix('aborted because new action of same type requested', prefix)
                    else:
                        self.set_ready(prefix=prefix)
                        self.print_with_prefix('aborted', prefix)
                    action_server.set_aborted(r)
                elif self.is_finish(state):
                    r.error = result_type.SUCCESS
                    r.ids = self.shelf_to_layer[data.id]
                    self.set_ready(prefix=prefix)
                    action_server.set_succeeded(r)
                    self.print_with_prefix('finished successful', prefix)
            else:
                r.error = result_type.INVALID_ID
                r.error_msg = 'invalid shelf id: {}'.format(data.id)
                self.set_ready(prefix=prefix)
                self.print_with_prefix('invalid id {}'.format(data.id), prefix)
                action_server.set_aborted(r)
        elif self.is_busy(state):
            self.set_abort(prefix)
            self.print_with_prefix('aborted because server busy', prefix)
            r.error = result_type.SERVER_BUSY
            r.error_msg = 'aborted because server busy: {}'.format(self.state_code_to_name(state))
            action_server.set_aborted(r)
        elif state == self.KILL_DETECT_SHELF_LAYER:
            r.error = result_type.SERVER_BUSY
            r.error_msg = 'aborted because server busy: {}'.format(self.state_code_to_name(state))
            self.set_ready(prefix=prefix)
            self.print_with_prefix('aborted because server busy', prefix)
            action_server.set_aborted(r)

    def detect_facings_cb(self, data):
        """
        :type data: DetectFacingsGoal
        """
        result_type = DetectFacingsResult
        action_server = self.detect_facings_as
        prefix = 'detect_facings'

        self.print_with_prefix('called', prefix)
        r = result_type()
        state = self.wait_for(
            [self.READY, self.KILL_DETECT_SHELF_LAYER, self.KILL_DETECT_FACINGS, self.KILL_COUNT_PRODUCTS,
             self.BUSY_DETECT_SHELF_LAYER, self.BUSY_COUNT_PRODUCTS, self.BUSY_DETECT_FACINGS], prefix)
        if state in [self.KILL_DETECT_SHELF_LAYER, self.READY, self.KILL_COUNT_PRODUCTS]:
            if data.id in self.layer_ids:
                self.print_with_prefix('started', prefix)
                self.put_state(self.BUSY_DETECT_FACINGS, prefix)
                state = self.wait_for([self.ABORT, self.FINISH], prefix)
                if self.is_abort(state):
                    r.error = 3
                    if action_server.new_goal:
                        self.set_ready(self.KILL_DETECT_FACINGS, prefix)
                        self.warn_with_prefix('aborted because new action of same type requested', prefix)
                    else:
                        self.set_ready(prefix=prefix)
                        self.print_with_prefix('aborted', prefix)
                    action_server.set_aborted(r)
                elif self.is_finish(state):
                    r.error = result_type.SUCCESS
                    r.ids = self.layer_to_facing[data.id]
                    self.set_ready(prefix=prefix)
                    action_server.set_succeeded(r)
                    self.print_with_prefix('finished successful', prefix)
            else:
                r.error = result_type.INVALID_ID
                r.error_msg = 'invalid layer id: {}'.format(data.id)
                self.set_ready(prefix=prefix)
                self.print_with_prefix('invalid id {}'.format(data.id), prefix)
                action_server.set_aborted(r)
        elif self.is_busy(state):
            self.set_abort(prefix)
            self.print_with_prefix('aborted because server busy', prefix)
            r.error = result_type.SERVER_BUSY
            r.error_msg = 'aborted because server busy: {}'.format(self.state_code_to_name(state))
            action_server.set_aborted(r)
        elif state == self.KILL_DETECT_FACINGS:
            r.error = result_type.SERVER_BUSY
            r.error_msg = 'aborted because server busy: {}'.format(self.state_code_to_name(state))
            self.set_ready(prefix=prefix)
            self.print_with_prefix('aborted because server busy', prefix)
            action_server.set_aborted(r)

    def count_products_cb(self, data):
        """
        :type data: CountProductsGoal
        """
        result_type = CountProductsResult
        action_server = self.count_products_as
        prefix = 'count_products'

        self.print_with_prefix('called', prefix)
        r = result_type()
        state = self.wait_for(
            [self.READY, self.KILL_DETECT_SHELF_LAYER, self.KILL_DETECT_FACINGS, self.KILL_COUNT_PRODUCTS,
             self.BUSY_DETECT_SHELF_LAYER, self.BUSY_COUNT_PRODUCTS, self.BUSY_DETECT_FACINGS], prefix)
        if state in [self.KILL_DETECT_SHELF_LAYER, self.READY, self.KILL_DETECT_FACINGS]:
            if data.id in self.facing_ids:
                self.print_with_prefix('started', prefix)
                # self.put_state(self.BUSY_COUNT_PRODUCTS, prefix)
                self.set_finish(prefix)
                state = self.wait_for([self.ABORT, self.FINISH], prefix)
                if self.is_abort(state):
                    r.error = 3
                    if action_server.new_goal:
                        self.set_ready(self.KILL_COUNT_PRODUCTS, prefix)
                        self.warn_with_prefix('aborted because new action of same type requested', prefix)
                    else:
                        self.set_ready(prefix=prefix)
                        self.print_with_prefix('aborted', prefix)
                    action_server.set_aborted(r)
                elif self.is_finish(state):
                    r.error = result_type.SUCCESS
                    r.count = self.product_counts[data.id]
                    self.set_ready(prefix=prefix)
                    action_server.set_succeeded(r)
                    self.print_with_prefix('finished successful', prefix)
            else:
                r.error = result_type.INVALID_ID
                r.error_msg = 'invalid facing id: {}'.format(data.id)
                self.set_ready(prefix=prefix)
                self.print_with_prefix('invalid id {}'.format(data.id), prefix)
                action_server.set_aborted(r)
        elif self.is_busy(state):
            self.set_abort(prefix)
            self.print_with_prefix('aborted because server busy', prefix)
            r.error = result_type.SERVER_BUSY
            r.error_msg = 'aborted because server busy: {}'.format(self.state_code_to_name(state))
            action_server.set_aborted(r)
        elif state == self.KILL_COUNT_PRODUCTS:
            r.error = result_type.SERVER_BUSY
            r.error_msg = 'aborted because server busy: {}'.format(self.state_code_to_name(state))
            self.set_ready(prefix=prefix)
            self.print_with_prefix('aborted because server busy', prefix)
            action_server.set_aborted(r)

    def cancel_cb(self):
        prefix = 'cancel_callback'
        self.print_with_prefix('called', prefix)
        state = self.get_state(prefix)
        if self.is_server_idle(state):
            self.put_state(state, prefix)
            self.print_with_prefix('server idle', prefix)
        elif self.is_busy(state):
            self.set_abort(prefix)
            # self.state_lock.task_done()
            self.print_with_prefix('aborting {}'.format(self.state_code_to_name(state)), prefix)
        else:
            self.put_state(state, prefix)
            self.warn_with_prefix('called in unexpected state', prefix)

    def get_state(self, prefix):
        state = self.state_lock.get(timeout=ONE_YEAR)
        # self.print_with_prefix('removed state token {}'.format(state), prefix)
        # if task_done:
        #     self.state_lock.task_done()
        return state

    def put_state(self, state, prefix=''):
        self.state_lock.put(state)
        # self.print_with_prefix('added state {}'.format(state), prefix)

    def is_ready(self, state):
        return state in {self.KILL_DETECT_SHELF_LAYER, self.KILL_DETECT_FACINGS, self.KILL_COUNT_PRODUCTS,
                         self.READY}

    def is_busy(self, state):
        return state in {self.BUSY_DETECT_SHELF_LAYER, self.BUSY_DETECT_FACINGS, self.BUSY_COUNT_PRODUCTS}

    def is_abort(self, state):
        return state == self.ABORT

    def is_finish(self, state):
        return state == self.FINISH

    def is_server_idle(self, state):
        return self.is_ready(state) or self.is_finish(state) or self.is_abort(state)

    def set_finish(self, prefix):
        self.put_state(self.FINISH, prefix)

    def set_ready(self, state=None, prefix=''):
        if state is None:
            self.put_state(self.READY, prefix)
        else:
            if state in [self.KILL_DETECT_SHELF_LAYER, self.KILL_COUNT_PRODUCTS, self.KILL_DETECT_FACINGS]:
                self.put_state(state, prefix)

    def set_kill_next(self, state, prefix):
        self.put_state(state, prefix)

    def set_busy(self, state, prefix):
        self.put_state(state, prefix)

    def set_abort(self, prefix):
        self.put_state(self.ABORT, prefix)

    def wait_for(self, events, prefix):
        state = None
        while not rospy.is_shutdown():
            state = self.check_state(events, prefix)
            if state is not None:
                break
            else:
                rospy.sleep(.5)
        # try:
        #     self.state_lock.task_done()
        # except:
        #     pass
        return state

    def check_state(self, events, prefix):
        state = self.get_state(prefix)
        if state in events:
            return state
        else:
            self.put_state(state, prefix)
            # self.state_lock.task_done()
            return None

    def state_code_to_name(self, code):
        if code in {self.READY, self.KILL_DETECT_SHELF_LAYER, self.KILL_DETECT_FACINGS, self.KILL_COUNT_PRODUCTS}:
            return 'ready'
        if code == self.BUSY_DETECT_SHELF_LAYER:
            return 'detecting shelf layers'
        if code == self.BUSY_DETECT_FACINGS:
            return 'detecting shelf facings'
        if code == self.BUSY_COUNT_PRODUCTS:
            return 'counting products'
        if code == self.ABORT:
            return 'ABORT'
        if code == self.FINISH:
            return 'FINISH'
        return 'UNKNOWN STATE'

    def print_with_prefix(self, msg, prefix):
        rospy.loginfo('[{}] {}'.format(prefix, msg))

    def warn_with_prefix(self, msg, prefix):
        rospy.logwarn('[{}] {}'.format(prefix, msg))


rospy.init_node('dummy_interface')
rospy.set_param('shelf_detection_path/position/x', 12.2)
rospy.set_param('shelf_detection_path/position/y', 6.78)
rospy.set_param('shelf_detection_path/position/z', 0.0)
rospy.set_param('shelf_detection_path/orientation/x', 0)
rospy.set_param('shelf_detection_path/orientation/y', 0)
rospy.set_param('shelf_detection_path/orientation/z', 1)
rospy.set_param('shelf_detection_path/orientation/w', 0)
rospy.set_param('shelf_detection_path/Torso_lin1', 0.0)
rospy.set_param('shelf_detection_path/Torso_lin2/gain', 0.3)
rospy.set_param('shelf_detection_path/Torso_rot_1', -1.13446)

rospy.set_param('detect_facing_path/start/position/x', 12.2)
rospy.set_param('detect_facing_path/start/position/y', 6.78)
rospy.set_param('detect_facing_path/start/position/z', 0.0)
rospy.set_param('detect_facing_path/start/orientation/x', 0)
rospy.set_param('detect_facing_path/start/orientation/y', 0)
rospy.set_param('detect_facing_path/start/orientation/z', 1)
rospy.set_param('detect_facing_path/start/orientation/w', 0)
rospy.set_param('detect_facing_path/start/Torso_lin1', 0.0)
rospy.set_param('detect_facing_path/start/Torso_lin2', 0.85)
rospy.set_param('detect_facing_path/start/Torso_rot_1', -1.65806)

rospy.set_param('detect_facing_path/end/position/x', 11.2)
rospy.set_param('detect_facing_path/end/position/y', 6.78)
rospy.set_param('detect_facing_path/end/position/z', 0.0)
rospy.set_param('detect_facing_path/end/orientation/x', 0)
rospy.set_param('detect_facing_path/end/orientation/y', 0)
rospy.set_param('detect_facing_path/end/orientation/z', 1)
rospy.set_param('detect_facing_path/end/orientation/w', 0)
rospy.set_param('detect_facing_path/end/Torso_lin1', 0.0)
rospy.set_param('detect_facing_path/end/Torso_lin2', 0.85)
rospy.set_param('detect_facing_path/end/Torso_rot_1', -1.65806)

rospy.set_param('count_products_posture/facing_92c6c2/position/x', 11.2)
rospy.set_param('count_products_posture/facing_92c6c2/position/y', 6.78)
rospy.set_param('count_products_posture/facing_92c6c2/position/z', 0.0)
rospy.set_param('count_products_posture/facing_92c6c2/orientation/x', 0)
rospy.set_param('count_products_posture/facing_92c6c2/orientation/y', 0)
rospy.set_param('count_products_posture/facing_92c6c2/orientation/z', 1)
rospy.set_param('count_products_posture/facing_92c6c2/orientation/w', 0)
rospy.set_param('count_products_posture/facing_92c6c2/Torso_lin1', 0.2)
rospy.set_param('count_products_posture/facing_92c6c2/Torso_lin2', 1.2)
rospy.set_param('count_products_posture/facing_92c6c2/Torso_rot_1', -1.13446)

rospy.set_param('count_products_posture/facing_77bce2/position/x', 11.28)
rospy.set_param('count_products_posture/facing_77bce2/position/y', 6.78)
rospy.set_param('count_products_posture/facing_77bce2/position/z', 0.0)
rospy.set_param('count_products_posture/facing_77bce2/orientation/x', 0)
rospy.set_param('count_products_posture/facing_77bce2/orientation/y', 0)
rospy.set_param('count_products_posture/facing_77bce2/orientation/z', 1)
rospy.set_param('count_products_posture/facing_77bce2/orientation/w', 0)
rospy.set_param('count_products_posture/facing_77bce2/Torso_lin1', 0.2)
rospy.set_param('count_products_posture/facing_77bce2/Torso_lin2', 1.2)
rospy.set_param('count_products_posture/facing_77bce2/Torso_rot_1', -1.13446)

rospy.set_param('count_products_posture/facing_c4a62e/position/x', 11.36)
rospy.set_param('count_products_posture/facing_c4a62e/position/y', 6.78)
rospy.set_param('count_products_posture/facing_c4a62e/position/z', 0.0)
rospy.set_param('count_products_posture/facing_c4a62e/orientation/x', 0)
rospy.set_param('count_products_posture/facing_c4a62e/orientation/y', 0)
rospy.set_param('count_products_posture/facing_c4a62e/orientation/z', 1)
rospy.set_param('count_products_posture/facing_c4a62e/orientation/w', 0)
rospy.set_param('count_products_posture/facing_c4a62e/Torso_lin1', 0.2)
rospy.set_param('count_products_posture/facing_c4a62e/Torso_lin2', 1.2)
rospy.set_param('count_products_posture/facing_c4a62e/Torso_rot_1', -1.13446)

DummyInterface()
rospy.loginfo('dummy interface started')
while not rospy.is_shutdown():
    rospy.sleep(1)
