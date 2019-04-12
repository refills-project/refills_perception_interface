from __future__ import division

import json
from random import sample

import rospy
import numpy as np

from copy import deepcopy

from collections import defaultdict, OrderedDict
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from refills_msgs.msg import Barcode
from rospy import ROSException
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker, MarkerArray
from rospkg import RosPack

from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.tfwrapper import transform_pose, lookup_transform

MAP = 'map'


class BarcodeDetector(object):
    def __init__(self, knowrob):
        """
        :type knowrob: KnowRob
        """
        self.knowrob = knowrob

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.marker_object_ns = 'barcode_object'
        self.marker_text_ns = 'barcode_text'

        # self.detector_topic = 'barcode_detector'
        self.detector_topic = 'barcode/pose'
        self.refills_models_path = 'package://refills_models/'

        self.object_color = ColorRGBA(0, 0, 0, 1)
        self.text_color = ColorRGBA(1, 1, 1, 1)
        self.object_scale = Vector3(.05, .05, .05)
        self.text_scale = Vector3(0, 0, .05)
        self.listen = False
        self.sub = rospy.Subscriber(self.detector_topic, Barcode, self.cb, queue_size=100)

    def start_listening(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        """
        self.shelf_layer_id = shelf_layer_id
        self.barcodes = defaultdict(list)
        self.shelf_width = self.knowrob.get_shelf_layer_width(shelf_layer_id)
        self.current_shelf_layer_width = self.knowrob.get_shelf_layer_width(shelf_layer_id)
        self.current_frame_id = self.knowrob.get_perceived_frame_id(self.shelf_layer_id)
        self.T_map___layer = lookup_transform(self.current_frame_id, 'map')
        self.listen = True

    def stop_listening(self):
        """
        :return: dict mapping barcode to PoseStamped
        :rtype: dict
        """
        self.listen = False
        barcodes = self.cluster()
        self.publish_as_marker(barcodes)
        rospy.loginfo('detected {} barcodes'.format(len(barcodes)))
        return barcodes

    def get_frame_id(self):
        return self.current_frame_id

    def cluster(self):
        """
        Replaces the list of poses where a barcode was seen with its average
        """
        barcodes = OrderedDict()
        for barcode, poses in sorted(self.barcodes.items(), key=lambda x: -len(x[1])):
            if len(poses) > 5: # FIXME magic number to expose
                positions = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses]
                position = np.mean(positions, axis=0)
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position = Point(*position)
                p.pose.orientation.w = 1
                barcodes[barcode] = p
        return barcodes

    def cb(self, data):
        """
        fills a dict that maps barcode to list of PoseStamped where it was seen.
        :type data: Barcode
        """
        if self.listen:
            p = transform_pose(MAP, data.barcode_pose)
            if p is not None:
                p.header.stamp = rospy.Time()
                if p is not None and self.barcode_on_shelf_layer(p):
                    if data.barcode[0] == '2':
                        self.barcodes[data.barcode[1:-1]].append(p)

    def barcode_on_shelf_layer(self, separator_pose, width_threshold=0.0, height_threshold=0.08):
        """
        :param separator_pose: pose of separator in floor frame
        :type separator_pose: PoseStamped
        :param width_threshold: all separators that are this close to the width edge are filtered.
        :type width_threshold: float
        :type height_threshold: float
        :return: bool
        """
        separator_pose = do_transform_pose(separator_pose, self.T_map___layer)
        x = separator_pose.pose.position.x
        z = separator_pose.pose.position.z
        return width_threshold <= x and x <= self.current_shelf_layer_width - width_threshold and \
               -height_threshold <= z and z <= height_threshold

    def publish_as_marker(self, barcodes):
        """
        Publishes barcodes as text marker.self.current_shelf_layer_width = self.knowrob.get_shelf_layer_width(shelf_layer_id)
        """
        ma = MarkerArray()
        frame_id = self.get_frame_id()
        for i, (barcode, pose) in enumerate(barcodes.items()):

            # text
            m = Marker()
            m.header.frame_id = frame_id
            m.ns = self.marker_text_ns
            m.id = int(barcode)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.text = barcode
            m.scale = self.text_scale
            m.color = self.text_color
            m.pose = deepcopy(pose.pose)
            m.pose.position.z += 0.07
            ma.markers.append(m)
        if len(ma.markers) > 0:
            self.marker_pub.publish(ma)

if __name__ == u'__main__':
    rospy.init_node('asdf')
    kr = KnowRob()
    b = BarcodeDetector(kr)
    # b.start_listening()
    rospy.sleep(5)
    b.cluster()