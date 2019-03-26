import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, TransformStamped, Pose, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_matrix
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3, do_transform_point
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener
import numpy as np

tfBuffer = None
tf_listener = None


def init(tf_buffer_size=15):
    """
    If you want to specify the buffer size, call this function manually, otherwise don't worry about it.
    :param tf_buffer_size: in secs
    :type tf_buffer_size: int
    """
    global tfBuffer, tf_listener
    tfBuffer = Buffer(rospy.Duration(tf_buffer_size))
    tf_listener = TransformListener(tfBuffer)
    rospy.sleep(5.0)


def transform_pose(target_frame, pose, transform=None):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type pose: PoseStamped
    :return: Transformed pose of None on loop failure
    :rtype: PoseStamped
    """
    if transform is None:
        global tfBuffer
        if tfBuffer is None:
            init()
        try:
            transform = tfBuffer.lookup_transform(target_frame,
                                                  pose.header.frame_id,  # source frame
                                                  pose.header.stamp,
                                                  rospy.Duration(5.0))
        except ExtrapolationException as e:
            rospy.logwarn(e)
            return None
    new_pose = do_transform_pose(pose, transform)
    return new_pose


def transform_vector(target_frame, vector):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type vector: Vector3Stamped
    :return: Transformed pose of None on loop failure
    :rtype: Vector3Stamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    try:
        transform = tfBuffer.lookup_transform(target_frame,
                                              vector.header.frame_id,  # source frame
                                              vector.header.stamp,
                                              rospy.Duration(5.0))
        new_pose = do_transform_vector3(vector, transform)
        return new_pose
    except ExtrapolationException as e:
        rospy.logwarn(e)


def transform_point(target_frame, point):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type point: PointStamped
    :return: Transformed pose of None on loop failure
    :rtype: PointStamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    try:
        transform = tfBuffer.lookup_transform(target_frame,
                                              point.header.frame_id,  # source frame
                                              point.header.stamp,
                                              rospy.Duration(5.0))
        new_pose = do_transform_point(point, transform)
        return new_pose
    except ExtrapolationException as e:
        rospy.logwarn(e)


def lookup_transform(target_frame, source_frame, time=rospy.Time()):
    """
    :type target_frame: str
    :type source_frame: str
    :return: Transform from target_frame to source_frame
    :rtype: PoseStamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    try:
        transform = tfBuffer.lookup_transform(target_frame, source_frame, time, rospy.Duration(5.0))
        return transform
    except:
        return None


def lookup_pose(target_frame, source_frame):
    """
    :type target_frame: str
    :type source_frame: str
    :return: target_frame <- source_frame
    :rtype: PoseStamped
    """
    p = PoseStamped()
    p.header.frame_id = source_frame
    p.pose.orientation.w = 1.0
    return transform_pose(target_frame, p)


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


def transformstamped_to_kdl(transformstamped):
    """Convert a geometry_msgs Transform message to a PyKDL Frame.

    :param transformstamped: The Transform message to convert.
    :type transformstamped: TransformStamped
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(transformstamped.transform.rotation.x,
                                                 transformstamped.transform.rotation.y,
                                                 transformstamped.transform.rotation.z,
                                                 transformstamped.transform.rotation.w),
                       PyKDL.Vector(transformstamped.transform.translation.x,
                                    transformstamped.transform.translation.y,
                                    transformstamped.transform.translation.z))

def msg_to_kdl(msg):
    if isinstance(msg, TransformStamped):
        return transformstamped_to_kdl(msg)
    elif isinstance(msg, PoseStamped):
        return posestamped_to_kdl(msg)
    else:
        raise TypeError('cant convert {} to kdl'.format(type(msg)))

def kdl_to_posestamped(frame, frame_id):
    """
    :type frame: PyKDL.Frame
    :rtype: PoseStamped
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
    return PoseStamped(pose=p, header=Header(frame_id=frame_id))