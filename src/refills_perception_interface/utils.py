from contextlib import contextmanager
from multiprocessing import Lock
import PyKDL
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import numpy as np
from tf.transformations import quaternion_from_matrix
import rospy


class TimeoutLock(object):
    def __init__(self):
        self._lock = Lock()

    def acquire(self, timeout=-1):
        return self._lock.acquire(timeout=timeout)

    @contextmanager
    def acquire_timeout(self, timeout):
        result = self._lock.acquire(timeout=timeout)
        yield result
        if result:
            self._lock.release()

    def release(self):
        self._lock.release()

def print_with_prefix(msg, prefix):
    rospy.loginfo('[{}] {}'.format(prefix, msg))

def warn_with_prefix(msg, prefix):
    rospy.logwarn('[{}] {}'.format(prefix, msg))

def error_with_refix(msg, prefix):
    rospy.logerr('[{}] {}'.format(prefix, msg))



def posestamped_to_kdl(posestamped):
    """Convert a geometry_msgs Transform message to a PyKDL Frame.
    :param posestamped: The Transform message to convert.
    :type posestamped: PoseStamped
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(posestamped.pose.orientation.x,
                                                 posestamped.pose.orientation.y,
                                                 posestamped.pose.orientation.z,
                                                 posestamped.pose.orientation.w),
                       PyKDL.Vector(posestamped.pose.position.x,
                                    posestamped.pose.position.y,
                                    posestamped.pose.position.z))


def kdl_to_pose(frame):
    """
    :type frame: PyKDL.Frame
    :rtype: Pose
    """
    p = Pose()
    p.position.x = frame.p[0]
    p.position.y = frame.p[1]
    p.position.z = frame.p[2]
    m = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2], 0],
                  [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2], 0],
                  [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2], 0],
                  [0, 0, 0, 1]])
    p.orientation = Quaternion(*quaternion_from_matrix(m))
    return p

import yaml
from collections import OrderedDict

def ordered_load(stream, Loader=yaml.Loader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass
    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)
    return yaml.load(stream, OrderedLoader)