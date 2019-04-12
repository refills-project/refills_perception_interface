import rospy
from iai_wsg_50_msgs.msg import PositionCmd, Status


class MoveGripper(object):
    def __init__(self):
        self.gripper_pub = rospy.Publisher('/wsg_50_driver/goal_position', PositionCmd, queue_size=10)
        self.status_sub = rospy.Subscriber('/wsg_50_driver/status', Status, self.status_cb, queue_size=10)
        self.force = 55
        self.speed = 50
        self.status = None # type: Status

    def status_cb(self, data):
        self.status = data

    def release(self):
        self.set_pose(self.get_gripper_pose() + .01)

    def open(self):
        self.set_pose(110)

    def close_gripper(self):
        self.set_pose(0)

    def is_goal_reached(self):
        return self.status and ('Target Pos reached' in self.status.status or self.status.speed == 0)

    def set_pose(self, width, timeout=5):
        cmd = PositionCmd()
        cmd.pos = width * 1000
        cmd.force = self.force
        cmd.speed = self.speed
        self.gripper_pub.publish(cmd)
        i = 0
        delta_t = 0.1
        while not self.is_goal_reached():
            rospy.sleep(delta_t)
            i += delta_t
            if delta_t > timeout:
                raise RuntimeError('gripper didn\'t reach goal')

    def get_gripper_pose(self):
        return rospy.wait_for_message('wsg_50/state', Status).width