import rospy

from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.tfwrapper import lookup_pose

db = '~/mongo_logs/2020-11-12_11-26-44_pp/roslog'

rospy.init_node('test')
knowrob = KnowRob(initial_mongo_db=db,
                  clear_roslog=True)
# knowrob = KnowRob(initial_mongo_db=None,
#                   clear_roslog=False)
shelf_ids = knowrob.get_shelf_system_ids(False)
print(shelf_ids)
for shelf_id in shelf_ids:
    print(knowrob.get_shelf_pose(shelf_id))
    perceived_frame_id = knowrob.get_perceived_frame_id(shelf_id)
    print(lookup_pose('map', perceived_frame_id))