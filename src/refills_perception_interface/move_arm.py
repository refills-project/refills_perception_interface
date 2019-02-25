import rospy
from geometry_msgs.msg import Quaternion, Point, PoseStamped, QuaternionStamped, PointStamped
from giskard_msgs.msg import Controller, ControllerListGoal, ControllerListAction, MoveResult
from sensor_msgs.msg import JointState
import numpy as np
from giskardpy.python_interface import GiskardWrapper


class MoveArm(object):
    def __init__(self, enabled=True, knowrob=None, avoid_self_collisinon=True):
        self.move_time_limit = 25
        self.enabled = enabled
        self.knowrob = knowrob
        self.giskard = GiskardWrapper()
        self.tip = 'camera_link'
        self.root = 'ur5_shoulder_link'
        self.trans_p_gain = 0.5
        self.rot_p_gain = 0.5
        self.trans_max_speed = 0.1
        self.rot_max_speed = 0.3
        self.avoid_self_collision = avoid_self_collisinon

        # TODO get this from param server of topic
        self.joint_names = ['ur5_shoulder_pan_joint',
                            'ur5_shoulder_lift_joint',
                            'ur5_elbow_joint',
                            'ur5_wrist_1_joint',
                            'ur5_wrist_2_joint',
                            'ur5_wrist_3_joint', ]

    def set_translation_goal(self, translation):
        goal_pose = PoseStamped()
        if isinstance(translation, PointStamped):
            goal_pose.header = translation.header
            goal_pose.pose.position = translation.point
        else:
            goal_pose = translation
        self.giskard.set_tranlation_goal(self.root, self.tip, goal_pose, self.trans_p_gain, self.trans_max_speed)

    def set_orientation_goal(self, orientation):
        goal_pose = PoseStamped()
        if isinstance(orientation, QuaternionStamped):
            goal_pose.header = orientation.header
            goal_pose.pose.orientation = orientation.quaternion
        else:
            goal_pose = orientation
        self.giskard.set_rotation_goal(self.root, self.tip, goal_pose, self.rot_p_gain, self.rot_max_speed)

    def set_and_send_cartesian_goal(self, goal_pose):
        self.set_translation_goal(goal_pose)
        self.set_orientation_goal(goal_pose)
        if not self.avoid_self_collision:
            self.giskard.disable_self_collision()
        return self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS

    def send_cartesian_goal(self):
        if self.enabled:
            if not self.avoid_self_collision:
                self.giskard.disable_self_collision()
            return self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS

    def set_and_send_joint_goal(self, joint_state):
        if self.enabled:
            self.giskard.set_joint_goal(joint_state)
            if not self.avoid_self_collision:
                self.giskard.disable_self_collision()
            return self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS

    def floor_detection_pose_right(self):
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

    def floor_detection_pose_left(self):
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
        return self.set_and_send_joint_goal(joint_state)

    def drive_pose(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = [
            -np.pi/2,
            -np.pi/2,
            -2.3,
            -np.pi/2,
            0,
            -np.pi/2,
        ]
        return self.set_and_send_joint_goal(joint_state)

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


if __name__ == '__main__':
    rospy.init_node('separator_detection_test')
    ma = MoveArm()
    # ma.floor_detection_pose2()
    ma.drive_pose()
    ma.floor_detection_pose2()
