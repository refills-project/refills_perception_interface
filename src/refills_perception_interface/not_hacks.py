from __future__ import division
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

from refills_perception_interface.tfwrapper import transform_pose, lookup_pose
import PyKDL


def add_bottom_layer_if_not_present(detected_shelf_layers, shelf_system_id, knowrob):
    """
    :type detected_shelf_layers: list
    :type knowrob: refills_perception_interface.knowrob_wrapper.KnowRob
    :rtype: list
    """

    if len(detected_shelf_layers) == 0 or min(detected_shelf_layers) > 0.3:
        if knowrob.is_7tile_system(shelf_system_id):
            detected_shelf_layers.insert(0, 0.2)
        else:
            detected_shelf_layers.insert(0, 0.15)
    return detected_shelf_layers


def add_separator_between_barcodes(separators, barcodes):
    """
    :type separators: list
    :type barcodes: list
    :rtype: tuple
    """
    separators = sorted(separators)
    barcodes = sorted(barcodes, key=lambda x: x[0])
    if len(barcodes) <= 1:
        return separators, barcodes

    new_separators = []
    for i_b in range(len(barcodes) - 1):
        barcode1 = barcodes[i_b][0]
        barcode2 = barcodes[i_b + 1][0]

        sbb = [s for s in separators if barcode1 <= s and s <= barcode2]
        if len(sbb) == 0:
            new_separators.append((barcode1 + barcode2) / 2)

    separators.extend(new_separators)
    separators = sorted(separators)
    return separators, barcodes


def add_edge_separators(separators):
    """
    :type separators: list
    :rtype: list
    """
    separators.append(0)
    separators.append(1)
    return sorted(separators)


def merge_close_separators(separators, threshold=0.03):
    """
    Merges separators that are closer than threshold together
    :type separators: list
    :type threshold: float
    :rtype: list
    """
    return merge_close_things(separators, threshold)


def merge_close_shelf_layers(shelf_layers, threshold=0.1):
    return merge_close_things(shelf_layers, threshold)


def merge_close_things(things, threshold):
    new_things = sorted(things)
    tmp = []
    while True:
        for i in range(len(new_things) - 1):
            s1 = new_things[i]
            s2 = new_things[i + 1]
            if abs(s1 - s2) < threshold:
                merged_separator = (s1 + s2) / 2
                tmp.append(merged_separator)
                tmp.extend(new_things[i + 2:])
                new_things = tmp
                tmp = []
                break
            else:
                tmp.append(s1)
        else:
            tmp.append(new_things[-1])
            return tmp


def update_shelf_system_pose(knowrob, top_layer_id, separators):
    """
    :type knowrob: refills_perception_interface.knowrob_wrapper.KnowRob
    :type shelf_system_id: str
    :type separators: list
    :return:
    """
    if not knowrob.is_bottom_layer(top_layer_id):
        return
    shelf_system_id = knowrob.get_shelf_system_from_layer(top_layer_id)
    shelf_system_frame_id = knowrob.get_object_frame_id(shelf_system_id)
    separators_in_system = [transform_pose(shelf_system_frame_id, p) for p in separators]

    separators_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in separators_in_system])

    separators_y = np.array(separators_xy)[:, 1].mean()
    T_system___layer = lookup_pose(shelf_system_frame_id, knowrob.get_perceived_frame_id(top_layer_id))
    y_offset = separators_y - T_system___layer.pose.position.y

    A = np.vstack([np.array(separators_xy)[:, 0], np.ones(separators_xy.shape[0])]).T
    y = separators_xy[:, 1]
    m, _ = np.linalg.lstsq(A, y, rcond=-1)[0]

    x = np.array([1, m, 0])
    x = x / np.linalg.norm(x)
    z = np.array([0, 0, 1])
    y = np.cross(z, x)

    q = PyKDL.Rotation(x[0], y[0], z[0],
                       x[1], y[1], z[1],
                       y[2], y[2], z[2]).GetQuaternion()
    offset = PoseStamped()
    offset.header.frame_id = shelf_system_frame_id
    offset.pose.position.y = y_offset
    offset.pose.orientation = Quaternion(*q)
    offset = transform_pose('map', offset)
    knowrob.belief_at_update(shelf_system_id, offset)
    rospy.sleep(0.5)
