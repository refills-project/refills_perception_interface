How to run demo:
start robot:
$ roslaunch refills_second_review bringup_everything.launch

start demo:
$ roscd refills_perception_interface
$ py.test test/test_perception_interface.py::TestPerceptionInterface::test_shop_scan

this will scan all shelves listed in 
refills_second_review/data/donbot_bremen/19_shelves_who_work.json
and in that order
PLCORHKS is left most one on window side
XWEYCFKB is the 3. on window side
XCBZVSEM is left most on isle
LOMKBJCG is middle one on isle

when changing the order, make sure that donbot has a straight line path between each pair.
