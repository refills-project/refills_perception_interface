from __future__ import division

import rospkg
from copy import deepcopy
from math import radians
import numpy as np

import PyKDL
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from refills_msgs.msg import FullBodyPosture, JointPosition, FullBodyPath
from geometry_msgs.msg import PoseStamped, Quaternion, Point, QuaternionStamped
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_about_axis
from trajectory_msgs.msg import JointTrajectory

from giskardpy.data_types import Quaternion
from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.tfwrapper import transform_pose, msg_to_kdl, lookup_pose, lookup_transform, \
    kdl_to_posestamped
import yaml

from refills_perception_interface.utils import kdl_to_pose

# TORSO_LIN1_UPPER_LIMIT = 0.6
# TORSO_LIN2_UPPER_LIMIT = 1.295
MIN_CAM_HEIGHT = 0.404
MAX_CAM_HEIGHT = 1.2

# CAM_IN_BASE_LINK = 0.862


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
        self.joint_names = ['ur5_shoulder_pan_joint',
                            'ur5_shoulder_lift_joint',
                            'ur5_elbow_joint',
                            'ur5_wrist_1_joint',
                            'ur5_wrist_2_joint',
                            'ur5_wrist_3_joint', ]

    def load_traj(self, name):
        rospack = rospkg.RosPack()
        path_to_pkg = rospack.get_path('refills_perception_interface')
        with open('{}/data/trajectories/refills_lab/{}.yaml'.format(path_to_pkg, name), 'r') as stream:
            try:
                traj = yaml.load(stream)
                msg = convert_dictionary_to_ros_message('control_msgs/FollowJointTrajectoryActionGoal',
                                                        traj)  # type: FollowJointTrajectoryActionGoal
                return msg.goal.trajectory
            except yaml.YAMLError as exc:
                print(exc)

    def get_shelf_layer_detection_traj(self):
        """
        :rtype: JointTrajectory
        """
        return self.load_traj('shelf_layer_path')

    def height_to_cam_pose(self, desired_height):
        """
        puts cam at desired height, keeping rot the same
        :type desired_height: float
        :return: goal height for lin joint 1, goal height for lin joint 2
        :rtype: PoseStamped
        """
        cam_pose = lookup_pose('base_footprint', 'camera_link')
        cam_pose.pose.position.z = max(MIN_CAM_HEIGHT, min(desired_height, MAX_CAM_HEIGHT))
        return cam_pose

    def get_cam_pose(self, height, rotation, left=True):
        fbp = FullBodyPosture()
        fbp.type = fbp.CAMERA
        fbp.camera_pos = self.height_to_cam_pose(height)
        if left:
            t_base_footprint___camera = PyKDL.Frame(PyKDL.Rotation(1, 0, 0,
                                                                   0, 0, 1,
                                                                   0, -1, 0))
        else:
            t_base_footprint___camera = PyKDL.Frame(PyKDL.Rotation(-1, 0, 0,
                                                                   0, 0, -1,
                                                                   0, -1, 0))
        t_camera___camera_goal = PyKDL.Frame(PyKDL.Rotation.RotX(rotation))
        t_base_footprint___camera_goal = t_base_footprint___camera * t_camera___camera_goal
        fbp.camera_pos.pose.orientation = kdl_to_pose(t_base_footprint___camera_goal).orientation
        # fbp.camera_pos.pose.orientation = transform_pose('camera_link', fbp.camera_pos).pose.orientation

        return fbp

    def is_left(self, shelf_system_id):
        return self.knowrob.is_left(shelf_system_id)

    def is_right(self, shelf_system_id):
        return self.knowrob.is_right(shelf_system_id)

    def cam_pose_in_front_of_shelf(self, shelf_system_id, x=0., y=-0.68, x_limit=0.1):
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

        # base_pose = self.cam_pose_to_base_pose(cam_pose)
        # return base_pose
        return cam_pose

    def cam_pose_to_base_pose(self, cam_pose):
        """
        :type cam_pose: PoseStamped
        :rtype: PoseStamped
        """
        T_shelf___cam_joint_g = msg_to_kdl(cam_pose)
        T_map___bfg = T_shelf___cam_joint_g * self.get_cam_in_base_footprint_kdl().Inverse()
        base_pose = kdl_to_posestamped(T_map___bfg, cam_pose.header.frame_id)
        return base_pose

    def get_cam_in_base_footprint_kdl(self):
        """
        :rtype: PyKDL.Frame
        """
        # if self.T_bf___cam_joint is None:
        T_bf___cam_joint = msg_to_kdl(lookup_transform('base_footprint', 'camera_link'))
        T_bf___cam_joint.p[2] = 0
        T_bf___cam_joint.M = PyKDL.Rotation()
        return T_bf___cam_joint

    def get_floor_detection_pose_left(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            0.0,
            -1.768,
            -0.51,
            -2.396,
            0.243438,
            -np.pi,
        ]
        return joint_state

    def get_floor_detection_pose_right(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -np.pi,
            -1.37,
            0.51,
            -0.72,
            -0.22,
            0,
        ]
        return joint_state

    def get_detect_shelf_layers_path(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: FullBodyPath
        """
        shelf_system_width = self.knowrob.get_shelf_layer_width(shelf_system_id)
        # shelf_system_height = self.knowrob.get_shelf_system_height(shelf_system_id)
        full_body_path = FullBodyPath()

        # calculate base pose

        if self.is_left(shelf_system_id):
            base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, shelf_system_width / 2)
            # base_pose = transform_pose('map', base_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.base_pos = deepcopy(base_pose)
            full_body_pose.base_pos.pose.position.y -= 0.2
            full_body_pose.type = FullBodyPosture.BASE
            full_body_path.postures.append(full_body_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.goal_joint_state = self.get_floor_detection_pose_left()
            full_body_pose.type = FullBodyPosture.JOINT
            full_body_path.postures.append(full_body_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.base_pos = base_pose
            full_body_pose.type = FullBodyPosture.BASE
            full_body_path.postures.append(full_body_pose)

            # full_body_pose = FullBodyPosture()
            # full_body_pose.type = FullBodyPosture.CAMERA
            # full_body_pose.camera_pos.header.frame_id = 'camera_link'
            # full_body_pose.camera_pos.pose.position = Point(0, 0, 0)
            # full_body_pose.camera_pos.pose.orientation = Quaternion(0, 0, 0, 1)
            # full_body_path.postures.append(full_body_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.type = FullBodyPosture.CAMERA
            full_body_pose.camera_pos.header.frame_id = 'camera_link'
            full_body_pose.camera_pos.pose.position = Point(0, 1, 0)
            full_body_pose.camera_pos.pose.orientation = Quaternion(0, 0, 0, 1)
            full_body_path.postures.append(full_body_pose)

            return full_body_path
        else:
            base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, shelf_system_width / 2)
            base_pose = transform_pose('map', base_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.base_pos = base_pose
            full_body_pose.type = FullBodyPosture.BASE
            full_body_path.postures.append(full_body_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.goal_joint_state = self.get_floor_detection_pose_right()
            full_body_pose.type = FullBodyPosture.JOINT
            full_body_path.postures.append(full_body_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.base_pos = base_pose
            full_body_pose.type = FullBodyPosture.BASE
            full_body_path.postures.append(full_body_pose)

            # full_body_pose = FullBodyPosture()
            # full_body_pose.type = FullBodyPosture.CAMERA
            # full_body_pose.camera_pos.header.frame_id = 'camera_link'
            # full_body_pose.camera_pos.pose.position = Point(0, 0, 0)
            # full_body_pose.camera_pos.pose.orientation = Quaternion(0, 0, 0, 1)
            # full_body_path.postures.append(full_body_pose)

            full_body_pose = FullBodyPosture()
            full_body_pose.type = FullBodyPosture.CAMERA
            full_body_pose.camera_pos.header.frame_id = 'camera_link'
            full_body_pose.camera_pos.pose.position = Point(0, 1, 0)
            full_body_pose.camera_pos.pose.orientation = Quaternion(0, 0, 0, 1)
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

        to_low_offset = 0.05
        other_offset = 0.05

        full_body_pose = FullBodyPosture()
        full_body_pose.type = FullBodyPosture.CAMERA

        if self.layer_too_low(shelf_layer_height):
            full_body_pose = self.get_cam_pose(shelf_layer_height + to_low_offset, radians(-25), self.is_left(shelf_system_id))
        else:
            full_body_pose = self.get_cam_pose(shelf_layer_height + other_offset, radians(0), self.is_left(shelf_system_id))

        full_body_path.postures.append(full_body_pose)

        # base poses
        # start base poses
        start_base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id)
        start_base_pose = transform_pose('map', start_base_pose)
        start_full_body_pose = FullBodyPosture()
        start_full_body_pose.type = FullBodyPosture.BASE
        start_full_body_pose.base_pos = start_base_pose

        end_base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, x=shelf_system_width)
        end_base_pose = transform_pose('map', end_base_pose)
        end_full_body_pose = FullBodyPosture()
        end_full_body_pose.type = FullBodyPosture.BASE
        end_full_body_pose.base_pos = end_base_pose

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
        counting_offset = 0.35
        # TODO tune this number
        torso_rot_1_height = shelf_layer_height + counting_offset
        torso_rot_1_height = max(MIN_CAM_HEIGHT, torso_rot_1_height)

        full_body_pose = self.get_cam_pose(torso_rot_1_height, radians(-25), self.is_left(shelf_system_id))

        facing_pose_on_layer = lookup_pose(shelf_layer_frame_id, facing_frame_id)

        base_pose = self.cam_pose_in_front_of_shelf(shelf_system_id, x=facing_pose_on_layer.pose.position.x)

        base_pose = transform_pose('map', base_pose)

        full_body_pose.type = full_body_pose.BOTH
        full_body_pose.base_pos = base_pose
        return full_body_pose
