import PyKDL
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, QuaternionStamped, PointStamped
from giskard_msgs.msg import MoveResult
from sensor_msgs.msg import JointState

import giskardpy.tfwrapper as tf
from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import transform_pose, lookup_transform
from refills_perception_interface.tfwrapper import msg_to_kdl, kdl_to_posestamped


class MoveArm(object):
    def __init__(self, enabled=True, knowrob=None, avoid_self_collisinon=True, tip='camera_link',
                 root='iiwa_link_0'):
        tf.init()
        self.move_time_limit = 25
        self.enabled = enabled
        self.knowrob = knowrob
        self.giskard = GiskardWrapper()
        self.tip = tip
        self.root = root
        self.trans_p_gain = 0.5
        self.rot_p_gain = 0.5
        self.trans_max_speed = 0.1
        self.rot_max_speed = 0.3
        self.base_link = 'base_link'
        # self.self_collision_min_dist = 0.03
        self.avoid_self_collision = avoid_self_collisinon

        # TODO get this from param server of topic
        self.joint_names = ['iiwa_joint_1',
                            'iiwa_joint_2',
                            'iiwa_joint_3',
                            'iiwa_joint_4',
                            'iiwa_joint_5',
                            'iiwa_joint_6',
                            'iiwa_joint_7']

    def set_translation_goal(self, translation):
        goal_pose = PoseStamped()
        if isinstance(translation, PointStamped):
            goal_pose.header = translation.header
            goal_pose.pose.position = translation.point
        else:
            goal_pose = translation
        self.giskard.set_translation_goal(root_link=self.root,
                                          tip_link=self.tip,
                                          goal_pose=goal_pose,
                                          max_velocity=self.trans_max_speed)

    def set_orientation_goal(self, orientation):
        goal_pose = PoseStamped()
        if isinstance(orientation, QuaternionStamped):
            goal_pose.header = orientation.header
            goal_pose.pose.orientation = orientation.quaternion
        else:
            goal_pose = orientation
        self.giskard.set_rotation_goal(goal_pose=goal_pose,
                                       root_link=self.root,
                                       tip_link=self.tip,
                                       max_velocity=self.rot_max_speed)

    def set_default_self_collision_avoidance(self):
        if not self.avoid_self_collision:
            self.giskard.allow_self_collision()
        # else:
        # self.giskard.set_self_collision_distance(self.self_collision_min_dist)
        # self.giskard.allow_collision(['ur5_wrist_3_link'], self.giskard.get_robot_name(), ['ur5_forearm_link'])

    def set_and_send_cartesian_goal(self, goal_pose):
        self.giskard.set_cart_goal(goal_pose=goal_pose,
                                   root_link=self.root,
                                   tip_link=self.tip)
        # self.set_orientation_goal(goal_pose)
        # self.set_default_self_collision_avoidance()
        return self.giskard.plan_and_execute().error_codes[0] == MoveResult.SUCCESS

    def send_cartesian_goal(self):
        if self.enabled:
            self.set_default_self_collision_avoidance()
            return self.giskard.plan_and_execute().error_codes[0] == MoveResult.SUCCESS

    def set_and_send_joint_goal(self, joint_state):
        if self.enabled:
            self.giskard.set_joint_goal(goal_state=joint_state)
            self.set_default_self_collision_avoidance()
            return self.giskard.plan_and_execute().error_codes[0] == MoveResult.SUCCESS

    def move_absolute(self, target_pose, retry=True):
        if self.enabled:
            self.giskard.set_cart_goal(goal_pose=target_pose,
                                       root_link='odom',
                                       tip_link=self.base_link)
            return self.giskard.plan_and_execute()
            # self.goal_pub.publish(target_pose)
            # goal = MoveBaseGoal()
            # goal.target_pose = target_pose
            # if self.knowrob is not None:
            #     self.knowrob.start_base_movement(self.knowrob.pose_to_prolog(target_pose))
            # while True:
            #     self.client.send_goal(goal)
            #     wait_result = self.client.wait_for_result(rospy.Duration(self.timeout))
            #     result = self.client.get_result()
            #     state = self.client.get_state()
            #     if wait_result and state == GoalStatus.SUCCEEDED:
            #         break
            #     if retry:
            #         cmd = raw_input('base movement did not finish in time, retry? [y/n]')
            #         retry = cmd == 'y'
            #     if not retry:
            #         print('movement did not finish in time')
            #         if self.knowrob is not None:
            #             self.knowrob.finish_action()
            #         raise TimeoutError()
            # if self.knowrob is not None:
            #     self.knowrob.finish_action()
            # return result

    def move_other_frame(self, target_pose, frame='camera_link', retry=True):
        if self.enabled:
            target_pose = self.cam_pose_to_base_pose(target_pose, frame)
            target_pose = transform_pose('map', target_pose)
            target_pose.pose.position.z = 0
            self.giskard.set_cart_goal(root_link='odom',
                                       tip_link=self.base_link,
                                       goal_pose=target_pose)
            return self.giskard.plan_and_execute()

            # self.goal_pub.publish(target_pose)
            # goal = MoveBaseGoal()
            # goal.target_pose = target_pose
            # if self.knowrob is not None:
            #     self.knowrob.start_base_movement(self.knowrob.pose_to_prolog(target_pose))
            # while True:
            #     self.client.send_goal(goal)
            #     wait_result = self.client.wait_for_result(rospy.Duration(self.timeout))
            #     result = self.client.get_result()
            #     state = self.client.get_state()
            #     if wait_result and state == GoalStatus.SUCCEEDED:
            #         break
            #     if retry:
            #         cmd = raw_input('base movement did not finish in time, retry? [y/n]')
            #         retry = cmd == 'y'
            #     if not retry:
            #         print('movement did not finish in time')
            #         if self.knowrob is not None:
            #             self.knowrob.finish_action()
            #         raise TimeoutError()
            # if self.knowrob is not None:
            #     self.knowrob.finish_action()
            # return result

    def cam_pose_to_base_pose(self, pose, frame):
        """
        :type pose: PoseStamped
        :rtype: PoseStamped
        """
        T_shelf___cam_joint_g = msg_to_kdl(pose)
        T_map___bfg = T_shelf___cam_joint_g * self.get_frame_in_base_footprint_kdl(frame).Inverse()
        base_pose = kdl_to_posestamped(T_map___bfg, pose.header.frame_id)
        return base_pose

    def get_frame_in_base_footprint_kdl(self, frame):
        """
        :rtype: PyKDL.Frame
        """
        # if self.T_bf___cam_joint is None:
        T_bf___cam_joint = msg_to_kdl(lookup_transform(self.base_link, frame))
        T_bf___cam_joint.p[2] = 0
        T_bf___cam_joint.M = PyKDL.Rotation()
        return T_bf___cam_joint

    def place_pose_left(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -1.57,
            -1.39,
            -2.4,
            0.46,
            1.57,
            -1.57,
        ]
        return self.set_and_send_joint_goal(joint_state)

    def place_pose_right(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -np.pi / 2,
            -2.44177755311,
            2.15026930371,
            0.291547812391,
            np.pi / 2,
            np.pi / 2,
        ]
        return self.set_and_send_joint_goal(joint_state)

    def floor_detection_pose_right(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            np.pi,
            -1.37,
            0.51,
            -0.72,
            -0.22,
            0,
        ]
        return self.set_and_send_joint_goal(joint_state)

    def floor_detection_pose_left(self):
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
        return self.set_and_send_joint_goal(joint_state)

    def drive_pose(self):
        return self.set_and_send_joint_goal({
            'iiwa_joint_1': 0.0,
            'iiwa_joint_2': -1.23,
            'iiwa_joint_3': 0.0,
            'iiwa_joint_4': 1.38999978643,
            'iiwa_joint_5': 0.0,
            'iiwa_joint_6': -0.85,
            'iiwa_joint_7': 0.0,
        })

    def pre_baseboard_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            0.0,
            -1.6460,
            -2.171,
            -0.85549,
            0.2181,
            -3.19172,
        ]
        return self.set_and_send_joint_goal(joint_state)

    def see_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            # -1.57114202181,
            # -1.44927913347,
            # -1.25000602404,
            # -4.01274043718,
            # -1.56251222292,
            # 1.62433183193,
            -np.pi / 2,
            -2.18,
            1.47,
            1.03,
            np.pi / 2,
            np.pi / 2,
        ]
        return self.set_and_send_joint_goal(joint_state)


# if __name__ == '__main__':
#     rospy.init_node('separator_detection_test')
#     ma = MoveArm()
#     # ma.floor_detection_pose2()
#     ma.drive_pose()
#     ma.floor_detection_pose2()
