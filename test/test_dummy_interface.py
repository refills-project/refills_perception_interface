import pytest
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from refills_msgs.msg import DetectShelfLayersAction, DetectShelfLayersGoal, DetectShelfLayersResult, DetectFacingsGoal, \
    DetectFacingsAction, CountProductsAction, DetectFacingsResult, CountProductsGoal, CountProductsResult
from refills_msgs.srv import QueryShelfLayersResponse, QueryShelfLayersRequest, FinishPerception, \
    FinishPerceptionRequest, FinishPerceptionResponse, QueryFacings, QueryFacingsResponse, QueryFacingsRequest, \
    QueryDetectShelfLayersPath, QueryDetectFacingsPath, QueryCountProductsPosture, QueryDetectShelfLayersPathResponse, \
    QueryDetectShelfLayersPathRequest, QueryDetectFacingsPathResponse, QueryDetectFacingsPathRequest, \
    QueryCountProductsPostureResponse, QueryCountProductsPostureRequest
from refills_msgs.srv import QueryShelfSystems, QueryShelfSystemsRequest, QueryShelfLayers
from refills_msgs.srv import QueryShelfSystemsResponse

NUM_SHELVES = 1
NUM_LAYER = 1
NUM_FACINGS = 3

DummyInterfaceNodeName = 'dummy_interface'


@pytest.fixture(scope='module')
def ros(request):
    print('init ros')
    rospy.init_node('tests')

    def kill_ros():
        print('shutdown ros')
        rospy.signal_shutdown('die')

    request.addfinalizer(kill_ros)


@pytest.fixture(scope='module')
def setup(request, ros):
    i = Interface()

    def reset_interface():
        i.reset()

    request.addfinalizer(reset_interface)
    return i


@pytest.fixture()
def interface(setup):
    setup.reset()
    return setup


class Interface(object):
    def __init__(self):
        rospy.init_node('tests')
        self.query_shelf_systems_srv = rospy.ServiceProxy(DummyInterfaceNodeName + '/query_shelf_systems',
                                                          QueryShelfSystems)
        self.query_shelf_layers_srv = rospy.ServiceProxy(DummyInterfaceNodeName + '/query_shelf_layers',
                                                         QueryShelfLayers)
        self.query_facings_srv = rospy.ServiceProxy(DummyInterfaceNodeName + '/query_facings',
                                                    QueryFacings)
        self.finish_perception_srv = rospy.ServiceProxy(DummyInterfaceNodeName + '/finish_perception',
                                                        FinishPerception)

        self.query_shelf_detection_path_srv = rospy.ServiceProxy(
            DummyInterfaceNodeName + '/query_detect_shelf_layers_path',
            QueryDetectShelfLayersPath)
        self.query_facing_detection_path_srv = rospy.ServiceProxy(
            DummyInterfaceNodeName + '/query_detect_facings_path',
            QueryDetectFacingsPath)
        self.query_product_counting_path_srv = rospy.ServiceProxy(
            DummyInterfaceNodeName + '/query_count_products_posture',
            QueryCountProductsPosture)

        self.detect_shelves_ac = SimpleActionClient(DummyInterfaceNodeName + '/detect_shelf_layers',
                                                    DetectShelfLayersAction)
        self.detect_facings_ac = SimpleActionClient(DummyInterfaceNodeName + '/detect_facings', DetectFacingsAction)
        self.count_products_ac = SimpleActionClient(DummyInterfaceNodeName + '/count_products', CountProductsAction)
        rospy.sleep(.5)

    def reset(self):
        self.cancel_detect_shelf_layers()
        self.cancel_detect_facings()
        self.cancel_count_products()

    # -------------------------------------------------------------other------------------------------------------------
    def query_shelf_systems(self):
        r = self.query_shelf_systems_srv.call(QueryShelfSystemsRequest())  # type: QueryShelfSystemsResponse
        assert len(r.ids) > 0
        return r.ids

    def finish_perception(self, expected_error=FinishPerceptionResponse.SUCCESS):
        r = self.finish_perception_srv.call(FinishPerceptionRequest())
        assert r.error == expected_error
        return r

    def query_detect_shelf_layers_path(self, shelf_id, expected_error=QueryDetectShelfLayersPathResponse.SUCCESS):
        r = self.query_shelf_detection_path_srv.call(QueryDetectShelfLayersPathRequest(id=shelf_id))
        assert r.error == expected_error
        if expected_error == QueryDetectShelfLayersPathResponse.SUCCESS:
            assert len(r.path.postures) > 0
            number_of_joints = -1
            for posture in r.path.postures:
                assert len(posture.joints) > 0
                if number_of_joints == -1:  # all postures should have the same number of joints
                    number_of_joints = len(posture.joints)
                assert len(posture.joints) == number_of_joints
        return r.path

    def query_detect_facings_path(self, layer_id, expected_error=QueryDetectFacingsPathResponse.SUCCESS):
        r = self.query_facing_detection_path_srv.call(QueryDetectFacingsPathRequest(id=layer_id))
        assert r.error == expected_error
        if expected_error == QueryDetectFacingsPathResponse.SUCCESS:
            assert len(r.path.postures) > 0
            number_of_joints = -1
            for posture in r.path.postures:
                assert len(posture.joints) > 0
                if number_of_joints == -1:  # all postures should have the same number of joints
                    number_of_joints = len(posture.joints)
                assert len(posture.joints) == number_of_joints
        return r.path

    def query_count_products_posture(self, facing_id, expected_error=QueryCountProductsPostureResponse.SUCCESS):
        r = self.query_product_counting_path_srv.call(QueryCountProductsPostureRequest(id=facing_id))
        assert r.error == expected_error
        if expected_error == QueryCountProductsPostureResponse.SUCCESS:
            assert len(r.posture.joints) > 0
        return r.posture

    # -------------------------------------------------------layer------------------------------------------------------
    def query_shelf_layers(self, shelf_id, expected_error=QueryShelfLayersResponse.SUCCESS):
        """
        :param shelf_id: str
        :rtype: QueryShelfLayersResponse
        """
        r = self.query_shelf_layers_srv.call(QueryShelfLayersRequest(id=shelf_id))
        assert r.error == expected_error
        return r.ids

    def start_detect_shelf_layers(self, shelf_id):
        goal = DetectShelfLayersGoal()
        goal.id = shelf_id
        self.detect_shelves_ac.send_goal(goal)
        rospy.sleep(.5)

    def get_detect_shelf_layers_result(self, expected_state=GoalStatus.SUCCEEDED,
                                       expected_error=DetectShelfLayersResult.SUCCESS):
        self.detect_shelves_ac.wait_for_result()
        assert self.detect_shelves_ac.get_state() == expected_state
        r = self.detect_shelves_ac.get_result()
        assert r.error == expected_error
        return r

    def detect_shelf_layers(self):
        shelf_ids = self.query_shelf_systems()
        result_dict = {}
        for shelf_id in shelf_ids:
            self.start_detect_shelf_layers(shelf_id)
            self.finish_perception()
            r = self.get_detect_shelf_layers_result()
            result_dict[shelf_id] = r.ids
            assert len(r.ids) > 0
        return result_dict

    def cancel_detect_shelf_layers(self):
        self.detect_shelves_ac.cancel_all_goals()

    # ------------------------------------------------------facing------------------------------------------------------
    def start_detect_facings(self, layer_id):
        goal = DetectFacingsGoal()
        goal.id = layer_id
        self.detect_facings_ac.send_goal(goal)
        rospy.sleep(.5)

    def get_detect_facings_result(self, expected_state=GoalStatus.SUCCEEDED,
                                  expected_error=DetectFacingsResult.SUCCESS):
        self.detect_facings_ac.wait_for_result()
        assert self.detect_facings_ac.get_state() == expected_state
        r = self.detect_facings_ac.get_result()
        assert r.error == expected_error
        return r

    def query_facings(self, layer_id, expected_error=QueryFacingsResponse.SUCCESS):
        r = self.query_facings_srv.call(QueryFacingsRequest(id=layer_id))
        assert r.error == expected_error
        return r.ids

    def detect_facings(self, layer_id):
        self.start_detect_facings(layer_id)
        self.finish_perception()
        r = self.get_detect_facings_result()
        assert len(r.ids) > 0
        return r.ids

    def cancel_detect_facings(self):
        self.detect_facings_ac.cancel_all_goals()

    # ------------------------------------------------------counting----------------------------------------------------
    def start_count_products(self, facing_id):
        goal = CountProductsGoal()
        goal.id = facing_id
        self.count_products_ac.send_goal(goal)
        rospy.sleep(.5)

    def get_count_products_result(self, expected_state=GoalStatus.SUCCEEDED,
                                  expected_error=CountProductsResult.SUCCESS):
        self.count_products_ac.wait_for_result()
        assert self.count_products_ac.get_state() == expected_state
        r = self.count_products_ac.get_result()
        assert r.error == expected_error
        return r

    def count_products(self, facing_id):
        self.start_count_products(facing_id)
        r = self.get_count_products_result()
        return r.count

    def cancel_count_products(self):
        self.count_products_ac.cancel_all_goals()


class TestDummyInterface(object):

    # -------------------------------------------------------other------------------------------------------------------
    def test_query_shelf_systems(self, interface):
        interface.query_shelf_systems()

    def test_query_shelf_systems_trice(self, interface):
        shelf_systems = interface.query_shelf_systems()
        assert shelf_systems == interface.query_shelf_systems()
        assert shelf_systems == interface.query_shelf_systems()

    def test_finish_perception_no_job(self, interface):
        interface.finish_perception(expected_error=FinishPerceptionResponse.NO_RUNNING_JOB)

    # -------------------------------------------------------layer------------------------------------------------------
    def test_query_shelf_layers_invalid_id(self, interface):
        interface.query_shelf_layers('', QueryFacingsResponse.INVALID_ID)

    def test_query_shelf_layers(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            layers = interface.query_shelf_layers(shelf_id)
            assert len(layers) == NUM_LAYER

    def test_query_shelf_layers_trice(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            layers = interface.query_shelf_layers(shelf_id)
            assert len(layers) == NUM_LAYER
            assert layers == interface.query_shelf_layers(shelf_id)
            assert layers == interface.query_shelf_layers(shelf_id)

    def test_detect_shelf_layers_invalid_id(self, interface):
        interface.start_detect_shelf_layers('')
        interface.get_detect_shelf_layers_result(expected_state=GoalStatus.ABORTED,
                                                 expected_error=DetectShelfLayersResult.INVALID_ID)

    def test_detect_shelf_layers(self, interface):
        interface.detect_shelf_layers()

    def test_detect_shelf_layers_trice(self, interface):
        shelves_to_layer = interface.detect_shelf_layers()
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            assert shelves_to_layer[shelf_id] == interface.query_shelf_layers(shelf_id)
            assert shelves_to_layer[shelf_id] == interface.query_shelf_layers(shelf_id)

    def test_cancel_detect_shelf_layers(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.cancel_detect_shelf_layers()
            interface.get_detect_shelf_layers_result(expected_state=GoalStatus.ABORTED,
                                                     expected_error=DetectShelfLayersResult.ABORTED)

    def test_cancel_detect_shelf_layers2(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.cancel_detect_shelf_layers()
            interface.get_detect_shelf_layers_result(expected_state=GoalStatus.ABORTED,
                                                     expected_error=DetectShelfLayersResult.ABORTED)
            interface.start_detect_shelf_layers(shelf_id)
            interface.finish_perception()
            interface.get_detect_shelf_layers_result()

    # -------------------------------------------------------facing-----------------------------------------------------
    def test_query_facings_invalid_id(self, interface):
        interface.query_facings('', QueryFacingsResponse.INVALID_ID)

    def test_query_facing_empty(self, interface):
        shelf_to_layer = interface.detect_shelf_layers()
        for shelf_id, layers in shelf_to_layer.items():
            for layer_id in layers:
                interface.query_facings(layer_id)

    def test_detect_facings(self, interface):
        shelf_to_layer = interface.detect_shelf_layers()
        for shelf_id, layers in shelf_to_layer.items():
            for layer_id in layers:
                facings = interface.detect_facings(layer_id)
                facings2 = interface.query_facings(layer_id)
                assert len(facings) > 0
                assert set(facings) == set(facings2)

    def test_detect_facings_invalid_id(self, interface):
        interface.start_detect_facings('')
        interface.get_detect_facings_result(expected_state=GoalStatus.ABORTED,
                                            expected_error=DetectShelfLayersResult.INVALID_ID)

    def test_cancel_detect_facings(self, interface):
        shelf_to_layer = interface.detect_shelf_layers()
        for shelf_id, layers in shelf_to_layer.items():
            for layer_id in layers:
                interface.start_detect_facings(layer_id)
                rospy.sleep(1)
                interface.cancel_detect_facings()
                interface.get_detect_facings_result(expected_state=GoalStatus.ABORTED,
                                                    expected_error=DetectShelfLayersResult.ABORTED)

    # -------------------------------------------------------counting---------------------------------------------------
    def test_count_products_invalid_id(self, interface):
        interface.start_count_products('')
        interface.get_count_products_result(expected_state=GoalStatus.ABORTED,
                                            expected_error=DetectShelfLayersResult.INVALID_ID)

    def test_count_products(self, interface):
        shelf_to_layer = interface.detect_shelf_layers()
        for shelf_id, layers in shelf_to_layer.items():
            for layer_id in layers:
                facings = interface.detect_facings(layer_id)
                for facing_id in facings:
                    interface.count_products(facing_id)

    # def test_cancel_count_products(self, interface):
    #     # FIXME
    #     shelf_to_layer = interface.detect_shelf_layers()
    #     for shelf_id, layers in shelf_to_layer.items():
    #         for layer_id in layers:
    #             facings = interface.detect_facings(layer_id)
    #             for facing_id in facings:
    #                 interface.start_count_products(facing_id)
    #                 interface.cancel_count_products()
    #                 interface.get_count_products_result(expected_state=GoalStatus.ABORTED,
    #                                                     expected_error=DetectShelfLayersResult.ABORTED)

    # -----------------------------------------------------paths--------------------------------------------------------
    def test_query_detect_shelf_layers_path_invalid_id(self, interface):
        interface.query_detect_shelf_layers_path('', QueryDetectShelfLayersPathResponse.INVALID_ID)

    def test_query_detect_facings_path_invalid_id(self, interface):
        interface.query_detect_facings_path('', QueryDetectShelfLayersPathResponse.INVALID_ID)

    def test_query_count_products_posture_invalid_id(self, interface):
        interface.query_count_products_posture('', QueryCountProductsPostureResponse.INVALID_ID)

    def test_query_detect_shelf_layers_path(self, interface):
        for shelf_id in interface.query_shelf_systems():
            path = interface.query_detect_shelf_layers_path(shelf_id)
            pass

    def test_query_detect_facings_path(self, interface):
        for shelf_id in interface.query_shelf_systems():
            for layer_id in interface.query_shelf_layers(shelf_id):
                interface.query_detect_facings_path(layer_id)

    def test_query_count_products_posture(self, interface):
        shelf_to_layer = interface.detect_shelf_layers()
        for shelf_id, layers in shelf_to_layer.items():
            for layer_id in layers:
                facings = interface.detect_facings(layer_id)
                for facing_id in facings:
                    interface.query_count_products_posture(facing_id)

    def test_shop_scan(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            path = interface.query_detect_shelf_layers_path(shelf_id)
            interface.detect_shelf_layers()
            layers = interface.query_shelf_layers(shelf_id)
            for layer_id in layers:
                path = interface.query_detect_facings_path(layer_id)
                interface.detect_facings(layer_id)
                facings = interface.query_facings(layer_id)
                for facing_id in facings:
                    posture = interface.query_count_products_posture(facing_id)
                    count = interface.count_products(facing_id)

    # -----------------------------------------------------test stupid calls--------------------------------------------
    def test_start_detection_twice0(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.start_detect_shelf_layers(shelf_id)
            interface.get_detect_shelf_layers_result(expected_error=DetectFacingsResult.SERVER_BUSY,
                                                     expected_state=GoalStatus.ABORTED)

    def test_start_detection_twice1(self, interface):
        # FIXME
        shelf_ids = interface.query_shelf_systems()
        layer_ids = interface.query_shelf_layers(shelf_ids[0])
        for layer_id in layer_ids:
            interface.start_detect_facings(layer_id)
            interface.start_detect_facings(layer_id)
            interface.get_detect_facings_result(expected_error=DetectFacingsResult.SERVER_BUSY,
                                                expected_state=GoalStatus.ABORTED)

    def test_start_detection_twice2(self, interface):
        shelf_ids = interface.query_shelf_systems()
        layer_ids = interface.query_shelf_layers(shelf_ids[0])
        facing_ids = interface.query_facings(layer_ids[0])
        for facing_id in facing_ids:
            interface.start_count_products(facing_id)
            interface.get_count_products_result(expected_error=DetectFacingsResult.SUCCESS,
                                                expected_state=GoalStatus.SUCCEEDED)
            interface.start_count_products(facing_id)
            interface.get_count_products_result(expected_error=DetectFacingsResult.SUCCESS,
                                                expected_state=GoalStatus.SUCCEEDED)

    def test_server_busy_layer_facing(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.start_detect_facings('')
            interface.get_detect_shelf_layers_result(expected_error=DetectShelfLayersResult.ABORTED,
                                                     expected_state=GoalStatus.ABORTED)
            interface.get_detect_facings_result(expected_error=DetectFacingsResult.SERVER_BUSY,
                                                expected_state=GoalStatus.ABORTED)

    def test_server_busy_facing_layer(self, interface):
        shelf_id = interface.query_shelf_systems()[0]
        layer_id = interface.query_shelf_layers(shelf_id)[0]
        facing_id = interface.query_facings(layer_id)[0]
        interface.start_detect_facings(layer_id)
        interface.start_detect_shelf_layers(shelf_id)
        interface.get_detect_facings_result(expected_error=DetectFacingsResult.ABORTED,
                                            expected_state=GoalStatus.ABORTED)
        interface.get_detect_shelf_layers_result(expected_error=DetectShelfLayersResult.SERVER_BUSY,
                                                 expected_state=GoalStatus.ABORTED)

    def test_server_busy_layer_count(self, interface):
        shelf_id = interface.query_shelf_systems()[0]
        layer_id = interface.query_shelf_layers(shelf_id)[0]
        facing_id = interface.query_facings(layer_id)[0]
        interface.start_detect_shelf_layers(shelf_id)
        interface.start_count_products(facing_id)
        interface.get_detect_shelf_layers_result(expected_error=DetectShelfLayersResult.ABORTED,
                                                 expected_state=GoalStatus.ABORTED)
        interface.get_count_products_result(expected_error=DetectFacingsResult.SERVER_BUSY,
                                            expected_state=GoalStatus.ABORTED)

    def test_server_busy_count_layer(self, interface):
        shelf_id = interface.query_shelf_systems()[0]
        layer_id = interface.query_shelf_layers(shelf_id)[0]
        facing_id = interface.query_facings(layer_id)[0]
        interface.start_count_products(facing_id)
        interface.start_detect_shelf_layers(shelf_id)
        interface.finish_perception()
        interface.get_count_products_result(expected_error=DetectFacingsResult.SUCCESS,
                                            expected_state=GoalStatus.SUCCEEDED)
        interface.get_detect_shelf_layers_result(expected_error=DetectShelfLayersResult.SUCCESS,
                                                 expected_state=GoalStatus.SUCCEEDED)

    def test_server_busy_facing_count(self, interface):
        shelf_id = interface.query_shelf_systems()[0]
        layer_id = interface.query_shelf_layers(shelf_id)[0]
        facing_id = interface.query_facings(layer_id)[0]
        interface.start_detect_facings(layer_id)
        interface.start_count_products(facing_id)
        interface.get_detect_facings_result(expected_error=DetectShelfLayersResult.ABORTED,
                                                 expected_state=GoalStatus.ABORTED)
        interface.get_count_products_result(expected_error=DetectFacingsResult.SERVER_BUSY,
                                            expected_state=GoalStatus.ABORTED)

    def test_server_busy_count_facing(self, interface):
        shelf_id = interface.query_shelf_systems()[0]
        layer_id = interface.query_shelf_layers(shelf_id)[0]
        facing_id = interface.query_facings(layer_id)[0]
        interface.start_count_products(facing_id)
        interface.start_detect_facings(layer_id)
        interface.finish_perception()
        interface.get_count_products_result(expected_error=DetectFacingsResult.SUCCESS,
                                            expected_state=GoalStatus.SUCCEEDED)
        interface.get_detect_facings_result(expected_error=DetectShelfLayersResult.SUCCESS,
                                                 expected_state=GoalStatus.SUCCEEDED)



    def test_detect_cancel_finish(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.cancel_detect_shelf_layers()
            interface.finish_perception(expected_error=FinishPerceptionResponse.NO_RUNNING_JOB)
            interface.get_detect_shelf_layers_result(expected_error=DetectFacingsResult.ABORTED,
                                                     expected_state=GoalStatus.ABORTED)

    def test_detect_finish_finish(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.finish_perception()
            interface.finish_perception(expected_error=FinishPerceptionResponse.NO_RUNNING_JOB)
            interface.get_detect_shelf_layers_result()

    def test_detect_cancel_cancel(self, interface):
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.cancel_detect_shelf_layers()
            interface.cancel_detect_shelf_layers()
            interface.get_detect_shelf_layers_result(expected_error=DetectShelfLayersResult.ABORTED,
                                                     expected_state=GoalStatus.ABORTED)

    def test_detect_finish_cancel(self, interface):
        # FIXME deadlock
        shelf_ids = interface.query_shelf_systems()
        for shelf_id in shelf_ids:
            interface.start_detect_shelf_layers(shelf_id)
            interface.finish_perception()
            interface.cancel_detect_shelf_layers()
            interface.get_detect_shelf_layers_result()
