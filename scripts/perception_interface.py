#!/usr/bin/env python
from __future__ import division
import functools
import py_trees
import rospy
from refills_msgs.msg import DetectShelfLayersAction, DetectFacingsAction, CountProductsAction
from refills_msgs.srv import FinishPerception, FinishPerceptionResponse
from py_trees import Behaviour, Blackboard, Status, BehaviourTree, Sequence, Selector

from refills_perception_interface.action_server_behavior import GoalReceived, PerceptionBehavior
from refills_perception_interface.count_products import CountProductsBehavior
from refills_perception_interface.detect_facings import DetectFacingsBehavior
from refills_perception_interface.detect_shelf_layers import DetectShelfLayersBehavior
from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.query_behavior import QueryBehavior
from refills_perception_interface.robosherlock_wrapper import FakeRoboSherlock, RoboSherlock
from refills_perception_interface.utils import TimeoutLock, print_with_prefix


def finish_perception_cb(data):
    """
    :type data: FinishPerceptionRequest
    :rtype: FinishPerceptionResponse
    """
    prefix = 'finish_perception'
    print_with_prefix('called', prefix)
    lock = Blackboard().lock # type: TimeoutLock
    result = FinishPerceptionResponse()
    with lock.acquire_timeout(0) as got_lock:
        if not got_lock:
            Blackboard().finished = True
            result.error = FinishPerceptionResponse.SUCCESS
        else:
            result.error = FinishPerceptionResponse.NO_RUNNING_JOB
            print_with_prefix('no running job', prefix)
    print_with_prefix('finished', prefix)
    return result

def grow_tree(debug=True):
    roboserlock_sim = rospy.get_param('~robosherlock_sim')
    b = Blackboard()
    b.finished = False
    b.lock = TimeoutLock()
    b.knowrob = KnowRob(initial_mongo_db='/home/arrina/mongo_logs/2020-11-11_11-24-18_pp/roslog',
                        clear_roslog=True)
    b.robot = rospy.get_param('~robot')
    if roboserlock_sim:
        b.robosherlock = FakeRoboSherlock(b.knowrob)
    else:
        b.robosherlock = RoboSherlock(b.knowrob)
    b.robot = rospy.get_param('~robot', 'donbot')

    finish_perception_srv = rospy.Service('~finish_perception', FinishPerception, finish_perception_cb)
    # ----------------------------------------------
    shelf_layer_as_name = '~detect_shelf_layers'

    shelf_layer1 = Sequence('layer detection 1')
    shelf_layer1.add_child(GoalReceived('got req', shelf_layer_as_name, DetectShelfLayersAction))

    shelf_layer1.add_child(DetectShelfLayersBehavior('shelf layer detection', shelf_layer_as_name))

    # ----------------------------------------------
    detect_facings_as_name = '~detect_facings'
    detect_facings1 = Sequence('detect facings 1')
    detect_facings1.add_child(GoalReceived('got req', detect_facings_as_name, DetectFacingsAction))

    detect_facings1.add_child(DetectFacingsBehavior('detect facings', detect_facings_as_name))

    # ----------------------------------------------
    count_products_as_name = '~count_products'
    count_products = Sequence('count products')
    count_products.add_child(GoalReceived('got req', count_products_as_name, CountProductsAction))

    count_products.add_child(CountProductsBehavior('count products', count_products_as_name))

    # ----------------------------------------------
    root = Selector(u'root')
    root.add_child(QueryBehavior('query'))
    root.add_child(shelf_layer1)
    root.add_child(detect_facings1)
    root.add_child(count_products)

    tree = BehaviourTree(root)

    if debug:
        # TODO create data folder if it does not exist
        def post_tick(snapshot_visitor, behaviour_tree):
            print(u'\n' + py_trees.display.ascii_tree(behaviour_tree.root,
                                                      snapshot_information=snapshot_visitor))

        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        tree.add_post_tick_handler(functools.partial(post_tick, snapshot_visitor))
        tree.visitors.append(snapshot_visitor)
        # render_dot_tree(root, name=path_to_data_folder + u'/tree')


    tree.setup(30)
    return tree


if __name__ == u'__main__':
    rospy.init_node('perception_interface')
    # rospy.sleep(5)
    debug = False
    if debug:
        tree_tick_rate = 2000
    else:
        tree_tick_rate = 20
    tree = grow_tree(debug)
    print('interface running')
    while not rospy.is_shutdown():
        try:
            tree.tick()
            rospy.sleep(tree_tick_rate/1000)
        except KeyboardInterrupt:
            break
    print(u'\n')
