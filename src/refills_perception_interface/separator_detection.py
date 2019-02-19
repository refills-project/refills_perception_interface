from __future__ import division, print_function

from sklearn.cluster import DBSCAN

import rospy
import numpy as np

from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from refills_msgs.msg import SeparatorArray
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_about_axis
from visualization_msgs.msg import MarkerArray

from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.tfwrapper import transform_pose
from refills_perception_interface.utils import print_with_prefix


class SeparatorClustering(object):
    prefix = 'separator detector'
    def __init__(self, knowrob):
        """
        :type knowrob: KnowRob
        """
        self.knowrob = knowrob
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.detections = []
        self.map_frame_id = 'map'
        self.separator_maker_color = ColorRGBA(.8, .8, .8, .8)
        self.separator_maker_scale = Vector3(.01, .5, .05)
        self.min_samples = 2
        self.max_dist = 0.015
        self.hanging = False
        self.listen = False
        self.separator_sub = rospy.Subscriber('separator_marker_detector_node/data_out', SeparatorArray,
                                              self.separator_cb,
                                              queue_size=10)

    def start_listening_separators(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        """
        self.hanging = False
        self.current_shelf_layer_id = shelf_layer_id
        self.detections = []
        self.marker_ns = 'separator_{}'.format(shelf_layer_id)
        self.current_shelf_layer_width = self.knowrob.get_shelf_layer_width(shelf_layer_id)
        self.current_frame_id = self.knowrob.get_perceived_frame_id(self.current_shelf_layer_id)
        self.listen = True
        print_with_prefix('started', self.prefix)

    def stop_listening(self):
        """
        :return: list of PoseStamped
        :rtype: list
        """
        self.listen = False
        separators = self.cluster()
        # separators.extend(self.get_edge_separators())
        print_with_prefix('stopped', self.prefix)
        return separators

    def get_frame_id(self):
        return self.current_frame_id

    # def get_edge_separators(self):
    #     """
    #     :str shelf_layer_id: str
    #     :return: list of PoseStamped with two separators, one on each end of the layer
    #     :rtype: list
    #     """
    #     separators = []
    #
    #     left_separator = PoseStamped()
    #     left_separator.header.frame_id = self.get_frame_id()
    #     left_separator.pose.orientation.w = 1
    #     separators.append(left_separator)
    #
    #     right_separator = PoseStamped()
    #     right_separator.header.frame_id = self.get_frame_id()
    #     right_separator.pose.position.x = self.current_shelf_layer_width
    #     right_separator.pose.orientation.w = 1
    #     separators.append(right_separator)
    #
    #     return separators

    def separator_cb(self, separator_array):
        """
        adds detected separators to self.detections
        :type separator_array: SeparatorArray
        """
        if self.listen:
            for separator in separator_array.separators:
                p = transform_pose(self.current_frame_id, separator.separator_pose)
                if p is not None and self.separator_on_shelf_layer(p):
                    self.detections.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])

    def separator_on_shelf_layer(self, separator_pose, width_threshold=0.03, height_threshold=0.05):
        """
        :param separator_pose: pose of separator in floor frame
        :type separator_pose: PoseStamped
        :param width_threshold: all separators that are this close to the width edge are filtered.
        :type width_threshold: float
        :type height_threshold: float
        :return: bool
        """
        x = separator_pose.pose.position.x
        z = separator_pose.pose.position.z
        return width_threshold <= x and x <= self.current_shelf_layer_width - width_threshold and \
               -height_threshold <= z and z <= height_threshold

    def cluster(self, visualize=False):
        """
        :param visualize: whether or not the debug plut should be shown !this might result in a exception because pyplot does not like multithreading!
        :type visualize: bool
        :return: list of PoseStamped
        :rtype: list
        """
        data = np.array(self.detections)
        separators = []
        old_frame_id = self.get_frame_id()
        if len(data) == 0:
            print_with_prefix('no separators detected', self.prefix)
        else:
            clusters = DBSCAN(eps=self.max_dist, min_samples=self.min_samples).fit(data)
            labels = np.unique(clusters.labels_)
            print_with_prefix('detected {} separators'.format(len(labels)), self.prefix)
            for i, label in enumerate(labels):
                if label != -1:
                    separator = PoseStamped()
                    separator.header.frame_id = old_frame_id
                    separator.pose.position = Point(*self.cluster_to_separator(data[clusters.labels_ == label]))
                    separator.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 2, [0, 0, 1]))
                    separators.append(separator)

            if visualize:
                detections = np.array(self.detections)
                self.visualize_detections(clusters.labels_, detections, self.pose_list_to_np(separators))
        return separators

    def cluster_to_separator(self, separator_cluster):
        """
        :param separator_cluster: 3*x
        :type separator_cluster: np.array
        :return: 3*1
        :rtype: np.array
        """
        return separator_cluster.mean(axis=0)

    def pose_list_to_np(self, poses):
        """
        :param poses: list of PoseStamped
        :type poses: list
        :return: 3*x numpy array
        :rtype: np.array
        """
        l = []
        for p in poses: # type: PoseStamped
            l.append([p.pose.position.x,
                      p.pose.position.y,
                      p.pose.position.z])
        return np.array(l)

    def visualize_detections(self, labels, detections, centers):
        """
        Creates a plot showing the separators, clusters and shit for debugging.
        :param labels: list of cluster label for each point in detections
        :type labels: list
        :param detections: contains 3d positions for all detected separators
        :type detections: np.array
        :param centers: array containing 3d positions of cluster centers
        :type centers: np.array
        :return:
        """
        import pylab as plt
        from mpl_toolkits.mplot3d import Axes3D # needed to make projection='3d' work

        ulabels = np.unique(labels)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for i, label in enumerate(ulabels):
            if i % 2 == 0:
                color = 'g'
            else:
                color = 'y'
            if label == -1:
                color = 'r'
            ax.scatter(detections[labels==label,0], detections[labels==label,1], detections[labels==label,2], c=color,
                       linewidth=0.0)

        ax.scatter(centers[:,0], centers[:,1], centers[:,2], c='k',
                   marker='x',s=80)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim(0,1)
        ax.set_ylim(-.5,.5)
        ax.set_zlim(-.5,.5)
        plt.show()


