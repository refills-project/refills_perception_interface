from multiprocessing import TimeoutError

import PyKDL
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf.transformations import quaternion_about_axis

from refills_perception_interface.tfwrapper import msg_to_kdl, kdl_to_posestamped, lookup_transform, transform_pose


class MoveBase(object):
    def __init__(self, move_base_action_name='nav_pcontroller/move_base', enabled=True, knowrob=None):
        # TODO use paramserver [low]
        self.enabled = enabled
        self.knowrob = knowrob
        self.client = actionlib.SimpleActionClient(move_base_action_name, MoveBaseAction)
        rospy.loginfo('connecting to {} ...'.format(move_base_action_name))
        self.client.wait_for_server()
        rospy.loginfo('connected to {}'.format(move_base_action_name))
        self.goal_pub = rospy.Publisher('move_base_goal', PoseStamped, queue_size=10)

        # self.laser_sub_front = rospy.Subscriber(rospy.get_param('~/laser/front', '/hokuyo_front/most_intense_throttle'),
        #                                         self.laser_cb, queue_size=10)
        # self.laser_sub_back = rospy.Subscriber(rospy.get_param('~/laser/back', '/hokuyo_back/most_intense_throttle'),
        #                                        self.laser_cb, queue_size=10)
        # self.min_dist_front = deque(maxlen=4)
        # self.min_dist_back = deque(maxlen=4)
        rospy.sleep(0.5)
        self.timeout = 60
        self.dist_to_shelfs = 1.4

    def move_absolute(self, target_pose, retry=True):
        if self.enabled:
            self.goal_pub.publish(target_pose)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            if self.knowrob is not None:
                self.knowrob.start_base_movement(self.knowrob.pose_to_prolog(target_pose))
            while True:
                self.client.send_goal(goal)
                wait_result = self.client.wait_for_result(rospy.Duration(self.timeout))
                result = self.client.get_result()
                state = self.client.get_state()
                if wait_result and state == GoalStatus.SUCCEEDED:
                    break
                if retry:
                    cmd = raw_input('base movement did not finish in time, retry? [y/n]')
                    retry = cmd == 'y'
                if not retry:
                    print('movement did not finish in time')
                    if self.knowrob is not None:
                        self.knowrob.finish_action()
                    raise TimeoutError()
            if self.knowrob is not None:
                self.knowrob.finish_action()
            return result

    def move_other_frame(self, target_pose, frame='camera_link', retry=True):
        if self.enabled:
            target_pose = self.cam_pose_to_base_pose(target_pose, frame)
            target_pose = transform_pose('map', target_pose)
            target_pose.pose.position.z = 0
            self.goal_pub.publish(target_pose)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            if self.knowrob is not None:
                self.knowrob.start_base_movement(self.knowrob.pose_to_prolog(target_pose))
            while True:
                self.client.send_goal(goal)
                wait_result = self.client.wait_for_result(rospy.Duration(self.timeout))
                result = self.client.get_result()
                state = self.client.get_state()
                if wait_result and state == GoalStatus.SUCCEEDED:
                    break
                if retry:
                    cmd = raw_input('base movement did not finish in time, retry? [y/n]')
                    retry = cmd == 'y'
                if not retry:
                    print('movement did not finish in time')
                    if self.knowrob is not None:
                        self.knowrob.finish_action()
                    raise TimeoutError()
            if self.knowrob is not None:
                self.knowrob.finish_action()
            return result

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
        T_bf___cam_joint = msg_to_kdl(lookup_transform('base_footprint', frame))
        T_bf___cam_joint.p[2] = 0
        T_bf___cam_joint.M = PyKDL.Rotation()
        return T_bf___cam_joint

    def move_absolute_xyz(self, frame_id, x, y, z, retry=True):
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.orientation = Quaternion(*quaternion_about_axis(z, [0, 0, 1]))
        return self.move_absolute(target_pose, retry)

    def move_relative(self, position=(0, 0, 0), orientation=(0, 0, 0, 1), retry=True):
        shelf = PoseStamped()
        header = Header()
        header.frame_id = 'base_footprint'
        shelf.header = header
        shelf.pose.position = Point(*position)
        shelf.pose.orientation = Quaternion(*orientation)
        self.move_absolute(shelf, retry)

    def STOP(self):
        self.client.cancel_goal()
        self.move_relative(retry=False)

    def laser_cb(self, data):
        min = rospy.get_param('/hokuyo_back/angle_min', -2.0)
        max = rospy.get_param('/hokuyo_back/angle_max', 2.0)
        data = LaserScan()
        for i, dist in enumerate(data.ranges):
            angle = min + i * data.angle_increment


    def get_c(self):
        pass

    def is_stuff_close(self, threshold=3):
        # TODO implement, maybe move somewhere else [low]
        rospy.logwarn('closest point not implemented')
        return False
