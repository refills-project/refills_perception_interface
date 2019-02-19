from __future__ import division
from math import radians
import numpy as np

import PyKDL
import rospy
from refills_msgs.msg import FullBodyPosture, JointPosition, FullBodyPath
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_about_axis

from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.tfwrapper import transform_pose, msg_to_kdl, lookup_pose, lookup_transform, \
    kdl_to_posestamped

TORSO_LIN1_UPPER_LIMIT = 0.6
TORSO_LIN2_UPPER_LIMIT = 1.295
MIN_CAM_HEIGHT = 0.404
CAM_IN_BASE_LINK = 0.862


# min is 1.6
# max is 2.3

class Paths(object):
    T_bf___cam_joint = None

    def __init__(self, knowrob):
        """
        :type knowrob: KnowRob
        """
        self.knowrob = knowrob
        self.ceiling_height = rospy.get_param('~ceiling_height')

    def _height_to_lin1_lin2(self, desired_height):
        """
        distributes desired height to the two linear joints equally.
        :type desired_height: float
        :return: goal height for lin joint 1, goal height for lin joint 2
        :rtype: tuple
        """
        desired_height = min(desired_height, self.ceiling_height)
        relative_height = desired_height - MIN_CAM_HEIGHT
        torso_lin_2 = min(relative_height, TORSO_LIN2_UPPER_LIMIT)
        torso_lin_1 = min(max(0, relative_height - torso_lin_2), TORSO_LIN1_UPPER_LIMIT)
        return torso_lin_1, torso_lin_2

    def _get_joints(self, height, rotation):
        torso_lin1 = JointPosition()
        torso_lin1.name = 'Torso_lin1'

        torso_lin2 = JointPosition()
        torso_lin2.name = 'Torso_lin2'

        torso_lin1.position, torso_lin2.position = self._height_to_lin1_lin2(height)

        torso_rot_1 = JointPosition()
        torso_rot_1.name = 'Torso_rot_1'
        torso_rot_1.position = rotation

        return [torso_lin1, torso_lin2, torso_rot_1]

    def is_left(self, shelf_system_id):
        return self.knowrob.is_left(shelf_system_id)

    def is_right(self, shelf_system_id):
        return self.knowrob.is_right(shelf_system_id)

    def cam_pose_in_front_of_shelf(self, shelf_system_id, x=0., y=-0.68, x_limit=0.2):
        """
        computes a goal for base_link such that torso_Schwenker_Cams is at x/y in front of shelf_system_id
        :type shelf_system_id: str
        :type x: float
        :type y: float
        :param x_limit: safety buffer range along x axis of shelf system
        :type x_limit: float
        :rtype: PoseStamped
        """
        width = self.knowrob.get_shelf_system_width(shelf_system_id)
        shelf_frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
        cam_pose = PoseStamped()
        cam_pose.header.frame_id = shelf_frame_id
        cam_pose.pose.position.x = max(0 + x_limit, min(width - x_limit, x))
        cam_pose.pose.position.y = y
        if self.is_left(shelf_system_id):
            cam_pose.pose.orientation = Quaternion(*quaternion_about_axis(0.0, [0, 0, 1]))
        else:
            cam_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))

        base_pose = self.cam_pose_to_base_pose(cam_pose)
        return base_pose

    def cam_pose_to_base_pose(self, cam_pose):
        """
        :type cam_pose: PoseStamped
        :rtype: PoseStamped
        """
        T_shelf___cam_joint_g = msg_to_kdl(cam_pose)
        T_map___bfg = T_shelf___cam_joint_g * self.get_cam_in_map_kdl().Inverse()
        base_pose = kdl_to_posestamped(T_map___bfg, cam_pose.header.frame_id)
        return base_pose

    def get_cam_in_map_kdl(self):
        """
        :rtype: PyKDL.Frame
        """
        if self.T_bf___cam_joint is None:
            self.T_bf___cam_joint = msg_to_kdl(lookup_transform('base_link', 'camera_link'))
            self.T_bf___cam_joint.p[2] = 0
            self.T_bf___cam_joint.M = PyKDL.Rotation()
        return self.T_bf___cam_joint

    def get_detect_shelf_layers_path(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: FullBodyPath
        """
        shelf_system_width = self.knowrob.get_shelf_layer_width(shelf_system_id)
        # shelf_system_height = self.knowrob.get_shelf_system_height(shelf_system_id)
        full_body_path = FullBodyPath()

        # calculate base pose
        base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, shelf_system_width / 2)
        base_pose = transform_pose('map', base_pose)

        # calculate torso joints
        layer1_rotation = 65
        horizontal = 90
        for (torso_extension, rotation) in (
                (MIN_CAM_HEIGHT + 0.1, layer1_rotation),
                (MIN_CAM_HEIGHT, layer1_rotation),
                (MIN_CAM_HEIGHT, horizontal),
                (self.ceiling_height, horizontal)
        ):
            full_body_pose = FullBodyPosture()
            full_body_pose.base_pos = base_pose
            # if self.is_left(shelf_system_id):
            #     full_body_pose.joints.extend(self._get_joints(torso_extension, radians(-rotation)))
            # else:
            #     full_body_pose.joints.extend(self._get_joints(torso_extension, radians(rotation)))
            full_body_path.postures.append(full_body_pose)

        return full_body_path

    def layer_too_low(self, shelf_layer_height):
        return shelf_layer_height < MIN_CAM_HEIGHT

    def get_detect_facings_path(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: FullBodyPath
        """
        shelf_system_id = self.knowrob.get_shelf_system_from_layer(shelf_layer_id)
        # shelf_system_frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
        shelf_system_width = self.knowrob.get_shelf_system_width(shelf_system_id)
        shelf_layer_frame_id = self.knowrob.get_perceived_frame_id(shelf_layer_id)
        full_body_path = FullBodyPath()

        # TODO consider left right cases

        # joints
        # TODO layer above
        shelf_layer_height = lookup_pose('map', shelf_layer_frame_id).pose.position.z
        # TODO tune this number
        torso_rot_1_height = shelf_layer_height

        to_low_offset = 0.1
        other_offset = 0.05

        # TODO tune this number
        if self.is_left(shelf_system_id):
            if self.layer_too_low(shelf_layer_height):
                joints = self._get_joints(MIN_CAM_HEIGHT + to_low_offset, radians(-65))
            else:
                joints = self._get_joints(torso_rot_1_height + other_offset, radians(-90))
        else:
            if self.layer_too_low(shelf_layer_height):
                joints = self._get_joints(MIN_CAM_HEIGHT + to_low_offset, radians(65))
            else:
                joints = self._get_joints(torso_rot_1_height + other_offset, radians(90))

        # start base poses

        start_base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id)
        start_base_pose = transform_pose('map', start_base_pose)

        start_full_body_pose = FullBodyPosture()
        start_full_body_pose.base_pos = start_base_pose
        start_full_body_pose.joints = joints

        # end base pose
        end_base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, x=shelf_system_width)
        end_base_pose = transform_pose('map', end_base_pose)

        end_full_body_pose = FullBodyPosture()
        end_full_body_pose.base_pos = end_base_pose
        end_full_body_pose.joints = joints

        if self.is_left(shelf_system_id):
            full_body_path.postures.append(end_full_body_pose)
            full_body_path.postures.append(start_full_body_pose)
        else:
            full_body_path.postures.append(start_full_body_pose)
            full_body_path.postures.append(end_full_body_pose)

        return full_body_path

    def get_count_product_posture(self, facing_id):
        """
        :type facing_id: str
        :rtype: FullBodyPosture
        """
        shelf_layer_id = self.knowrob.get_shelf_layer_from_facing(facing_id)
        shelf_system_id = self.knowrob.get_shelf_system_from_layer(shelf_layer_id)
        # shelf_system_width = self.knowrob.get_shelf_system_width(shelf_system_id)
        # shelf_system_frame_id = self.knowrob.get_perceived_frame_id(shelf_system_id)
        shelf_layer_frame_id = self.knowrob.get_perceived_frame_id(shelf_layer_id)
        facing_frame_id = self.knowrob.get_object_frame_id(facing_id)

        # get height of next layer
        # shelf_layer_above_id = self.knowrob.get_shelf_layer_above(shelf_layer_id)
        # if shelf_layer_above_id is not None:
        #     shelf_layer_above_frame_id = self.knowrob.get_perceived_frame_id(shelf_layer_above_id)
        #     height_of_next_layer = lookup_transform('map', shelf_layer_above_frame_id).pose.position.z
        # else:
        #     height_of_next_layer = self.knowrob.get_shelf_system_height(shelf_system_id)

        # joints
        shelf_layer_height = lookup_pose('map', shelf_layer_frame_id).pose.position.z
        counting_offset = 0.45
        # TODO tune this number
        torso_rot_1_height = shelf_layer_height + counting_offset
        torso_rot_1_height = max(MIN_CAM_HEIGHT, torso_rot_1_height)

        if self.is_left(shelf_system_id):
            joints = self._get_joints(torso_rot_1_height, radians(-65))
        else:
            joints = self._get_joints(torso_rot_1_height, radians(65))

        # TODO tune base bose
        facing_pose_on_layer = lookup_pose(shelf_layer_frame_id, facing_frame_id)

        base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, x=facing_pose_on_layer.pose.position.x)

        base_pose = transform_pose('map', base_pose)

        full_body_pose = FullBodyPosture()
        full_body_pose.base_pos = base_pose
        full_body_pose.joints = joints
        return full_body_pose
