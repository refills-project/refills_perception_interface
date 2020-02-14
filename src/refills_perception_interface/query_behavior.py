from multiprocessing import Lock
import datetime
import rospy
from refills_msgs.msg import FullBodyPosture, JointPosition
from refills_msgs.srv import QueryShelfSystems, QueryShelfLayers, QueryFacings, QueryDetectShelfLayersPath, \
    QueryDetectFacingsPath, QueryCountProductsPosture, QueryShelfSystemsResponse, QueryShelfLayersResponse, \
    QueryFacingsResponse, QueryDetectShelfLayersPathResponse, QueryDetectFacingsPathResponse, \
    QueryCountProductsPostureResponse
from geometry_msgs.msg import Point, Quaternion
from py_trees import Status
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker

from refills_perception_interface.MyBehavior import MyBahaviour
from refills_perception_interface.paths import Paths
from refills_perception_interface.utils import print_with_prefix


class QueryBehavior(MyBahaviour):
    """
    Behavior that handles all the queries.
    """

    prefix = 'query_behavior'
    def __init__(self, name=""):
        self.lock = Lock()
        super(QueryBehavior, self).__init__(name)

    def setup(self, timeout):
        self.paths = Paths(self.get_knowrob())
        self.query_shelf_systems_srv = rospy.Service('~query_shelf_systems', QueryShelfSystems,
                                                     self.query_shelf_systems_cb)
        self.query_shelf_layers_srv = rospy.Service('~query_shelf_layers', QueryShelfLayers, self.query_shelf_layers_cb)
        self.query_facings_srv = rospy.Service('~query_facings', QueryFacings, self.query_facings_cb)
        self.query_shelf_layer_detection_path_srv = rospy.Service('~query_detect_shelf_layers_path',
                                                                  QueryDetectShelfLayersPath,
                                                                  self.query_detect_shelf_layers_path_cb)
        self.query_facing_detection_path_srv = rospy.Service('~query_detect_facings_path', QueryDetectFacingsPath,
                                                             self.query_detect_facings_path_cb)
        self.query_product_counting_path_srv = rospy.Service('~query_count_products_posture',
                                                             QueryCountProductsPosture,
                                                             self.query_count_products_posture_cb)
        self.query_reset_beliefstate_srv = rospy.Service('~reset_beliefstate', Trigger, self.query_reset_beliefstate)

        self.visualization_marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # self.initial_beliefstate = rospy.get_param('~initial_beliefstate')

        self.get_knowrob().load_initial_beliefstate()
        return super(QueryBehavior, self).setup(timeout)

    def initialise(self):
        super(QueryBehavior, self).initialise()

    def terminate(self, new_status):
        super(QueryBehavior, self).terminate(new_status)

    def update(self):
        try:
            self.lock.release()
        except:
            return Status.FAILURE
        return Status.SUCCESS

    def wait_for_update(self):
        # TODO i dont know if that actually makes him wait.
        self.lock.acquire()

    def query_reset_beliefstate(self, req):
        """
        :type req: std_srvs.srv._Trigger.TriggerRequest
        :rtype: TriggerResponse
        """
        # self.initial_beliefstate = rospy.get_param('~initial_beliefstate')
        r = TriggerResponse()
        m = Marker()
        m.action = Marker.DELETEALL
        self.visualization_marker_pub.publish(m)
        rospy.sleep(.3)
        try:
            if not self.get_knowrob().stop_episode():
                rospy.logwarn('failed to stop episode')
            # now = datetime.datetime.now()
            # self.get_knowrob().save_beliefstate('/home/arrina/rosout_logs/belief_state_{}-{}-{}-{}-{}-{}.owl'.format(now.year, now.month, now.day, now.hour, now.minute, now.second))
        except Exception as e:
            self.get_knowrob().print_with_prefix('failed to save beliefstate before reset')
        r.success = self.get_knowrob().reset_beliefstate()
        return r

    def query_shelf_systems_cb(self, data):
        """
        :type data: QueryShelfSystemsRequest
        :rtype: QueryShelfSystemsResponse
        """
        prefix = 'query_shelf_systems'
        print_with_prefix('called', prefix)
        r = QueryShelfSystemsResponse()
        r.ids = self.get_knowrob().get_shelf_system_ids()
        self.wait_for_update()
        return r

    def query_shelf_layers_cb(self, data):
        """
        :type data: QueryShelfLayersRequest
        :rtype: QueryShelfLayersResponse
        """
        prefix = 'query_shelf_layers'
        print_with_prefix('called', prefix)
        r = QueryShelfLayersResponse()
        if self.get_knowrob().shelf_system_exists(data.id):
            r.error = QueryShelfLayersResponse.SUCCESS
            r.ids = self.get_knowrob().get_shelf_layer_from_system(data.id).keys()
        else:
            print_with_prefix('invalid id', prefix)
            r.error = QueryShelfLayersResponse.INVALID_ID
        self.wait_for_update()
        return r

    def query_facings_cb(self, data):
        """
        :type data: QueryFacingsRequest
        :rtype: QueryFacingsResponse
        """
        prefix = 'query_facings'
        print_with_prefix('called', prefix)
        r = QueryFacingsResponse()
        if self.get_knowrob().shelf_layer_exists(data.id):
            r.error = QueryFacingsResponse.SUCCESS
            r.ids = self.get_knowrob().get_facing_ids_from_layer(data.id).keys()
        else:
            print_with_prefix('invalid id', prefix)
            r.error = QueryFacingsResponse.INVALID_ID
        self.wait_for_update()
        return r

    def query_detect_shelf_layers_path_cb(self, data):
        """
        :type data: QueryDetectShelfLayersPathRequest
        :rtype: QueryDetectShelfLayersPathResponse
        """
        print_with_prefix('called', 'query_detect_shelf_layers_path')
        r = QueryDetectShelfLayersPathResponse()
        if self.get_knowrob().shelf_system_exists(data.id):
            r.error = QueryDetectShelfLayersPathResponse.SUCCESS
            # try:
            r.path = self.paths.get_detect_shelf_layers_path(data.id)
            # except:
            #     rospy.logerr('path error')
        else:
            r.error = QueryDetectShelfLayersPathResponse.INVALID_ID
        self.wait_for_update()
        return r

    def query_detect_facings_path_cb(self, data):
        """
        :type data: QueryDetectFacingsPathRequest
        :rtype: QueryDetectFacingsPathResponse
        """
        print_with_prefix('called', 'query_detect_facings_path')
        r = QueryDetectFacingsPathResponse()
        if self.get_knowrob().shelf_layer_exists(data.id):
            r.error = QueryDetectFacingsPathResponse.SUCCESS
            try:
                r.path = self.paths.get_detect_facings_path(data.id)
            except:
                rospy.logerr('path error')
        else:
            r.error = QueryDetectFacingsPathResponse.INVALID_ID
        self.wait_for_update()
        return r

    def query_count_products_posture_cb(self, data):
        """
        :type data: QueryCountProductsPostureRequest
        :rtype: QueryCountProductsPostureResponse
        """
        print_with_prefix('called', 'query_count_products_posture')
        r = QueryCountProductsPostureResponse()
        if self.get_knowrob().facing_exists(data.id):
            r.error = QueryCountProductsPostureResponse.SUCCESS
            # try:
            r.posture = self.paths.get_count_product_posture(data.id)
            # except:
            #     rospy.logerr('path error')

        else:
            r.error = QueryCountProductsPostureResponse.INVALID_ID
        self.wait_for_update()
        return r