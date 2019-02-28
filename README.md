py.test test/test_perception_interface.py::TestPerceptionInterface::test_shop_scan






This is a dummy interface according to refills interface spec v1.

Clear beliefstate and reload initial beliefstate:
rosservice call /perception_interface/reset_beliefstate "{}"

How to run:
rosrun refills_perception_interface dummy_interface.py
roslaunch refills_perception_interface interface.launch

How to run all tests (tests should work for the dummy interface as well as the real one):
py.test test/test_dummy_interface.py
py.test test/test_perception_interface.py

How to run individual tests:
py.test test/test_dummy_interface.py::TestDummyInterface::test_shop_scan
py.test test/test_perception_interface.py::TestDummyInterface::test_shop_scan_without_path

Example store scanning using terminal

1. get all shelf systems
2. for each system:
3.		ask for shelf layer detection path
4. 		start shelf layer detection
5. 		call finish perception service
6.		ask for shelf layers (optional as the action server returns them as well)
7.		for each layer:
8.			ask for facing detection path
9.			start facing detection
10.			call finish perception service
11.			ask for facings (optional as the action server returns them as well)
12.			for each facing:
13.				ask for counting posture
14.				start counting
15.				call finish perception service


Here is me doing one loop of this using the terminal:

1. get all shelf systems

$ rosservice call /dummy_interface/query_shelf_systems "{}" 
ids: [shelf_535dc8, shelf_3c0499, shelf_859e0f]

------------------------------------------------------------------------------------------------

3.		ask for shelf layer detection path

$ rosservice call /dummy_interface/query_detect_shelf_layers_path "id: 'shelf_535dc8'" 
path: 
  postures: 
    - 
      base_pos: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "shelf_535dc8"
        pose: 
          position: 
            x: 0.5
            y: -1.0
            z: 0.0
          orientation: 
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
      joints: 
        - 
          name: "torso_lin_1"
          position: 0.0
        - 
          name: "torso_lin_2"
          position: 0.0
        - 
          name: "base_of_schwenker_joint"
          position: 0.0
        - 
          name: "torso_rot_1"
          position: 0.0
    - 
      base_pos: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        pose: 
          position: 
            x: 0.0
            y: 0.0
            z: 0.0
          orientation: 
            x: 0.0
            y: 0.0
            z: 0.0
            w: 0.0
      joints: 
        - 
          name: "torso_lin_1"
          position: 0.0
        - 
          name: "torso_lin_2"
          position: 0.0
        - 
          name: "base_of_schwenker_joint"
          position: 0.0
        - 
          name: "torso_rot_1"
          position: 0.0
    - 
      base_pos: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        pose: 
          position: 
            x: 0.0
            y: 0.0
            z: 0.0
          orientation: 
            x: 0.0
            y: 0.0
            z: 0.0
            w: 0.0
      joints: 
        - 
          name: "torso_lin_1"
          position: 0.0
        - 
          name: "torso_lin_2"
          position: 0.0
        - 
          name: "base_of_schwenker_joint"
          position: 0.0
        - 
          name: "torso_rot_1"
          position: 0.0
error: 0

------------------------------------------------------------------------------------------------

4. 		start shelf layer detection

$ rostopic pub /dummy_interface/detect_shelf_layers/goal refills_msgs/DetectShelfLayersActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  id: 'shelf_535dc8'" 

------------------------------------------------------------------------------------------------

5. 		call finish perception service

$ rosservice call /dummy_interface/finish_perception "{}" 
error: 0
error_msg: ''

------------------------------------------------------------------------------------------------

6.		ask for shelf layers (optional as the action server returns them as well)

$ rosservice call /dummy_interface/query_shelf_layers "id: 'shelf_535dc8'" 
ids: [layer_faa57d, layer_36ce70, layer_3accc2]
error: 0

------------------------------------------------------------------------------------------------

8.			ask for facing detection path

$ rosservice call /dummy_interface/query_detect_facings_path "id: 'layer_faa57d'" 
path: 
  postures: 
    - 
      base_pos: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "layer_faa57d"
        pose: 
          position: 
            x: 0.0
            y: -0.5
            z: 0.0
          orientation: 
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
      joints: 
        - 
          name: "torso_lin_1"
          position: 0.0
        - 
          name: "torso_lin_2"
          position: 0.0
        - 
          name: "base_of_schwenker_joint"
          position: 0.0
        - 
          name: "torso_rot_1"
          position: 0.0
    - 
      base_pos: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "layer_faa57d"
        pose: 
          position: 
            x: 1.0
            y: -0.5
            z: 0.0
          orientation: 
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
      joints: 
        - 
          name: "torso_lin_1"
          position: 0.0
        - 
          name: "torso_lin_2"
          position: 0.0
        - 
          name: "base_of_schwenker_joint"
          position: 0.0
        - 
          name: "torso_rot_1"
          position: 0.0
error: 0

------------------------------------------------------------------------------------------------

9.			start facing detection

$ rostopic pub /dummy_interface/detect_facings/goal refills_msgs/DetectFacingsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  id: 'layer_faa57d'"

------------------------------------------------------------------------------------------------

10.			call finish perception service

$ rosservice call /dummy_interface/finish_perception "{}" 
error: 0
error_msg: ''

------------------------------------------------------------------------------------------------

11.			ask for facings (optional as the action server returns them as well)

$ rosservice call /dummy_interface/query_facings "id: 'layer_faa57d'" 
ids: [facing_92c6c2, facing_77bce2, facing_c4a62e]
error: 0

------------------------------------------------------------------------------------------------

13.				ask for counting posture

$ rosservice call /dummy_interface/query_count_products_posture "id: 'facing_92c6c2'" 
posture: 
  base_pos: 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "facing_92c6c2"
    pose: 
      position: 
        x: 0.0
        y: -0.5
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  joints: 
    - 
      name: "torso_lin_1"
      position: 0.0
    - 
      name: "torso_lin_2"
      position: 0.0
    - 
      name: "base_of_schwenker_joint"
      position: 0.0
    - 
      name: "torso_rot_1"
      position: 0.0
error: 0

------------------------------------------------------------------------------------------------

14.				start counting

$ rostopic pub /dummy_interface/count_products/goal refills_msgs/CountProductsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  id: 'facing_92c6c2'" 

------------------------------------------------------------------------------------------------

15.				call finish perception service

$ rosservice call /dummy_interface/finish_perception "{}" 
error: 0
error_msg: ''

------------------------------------------------------------------------------------------------
