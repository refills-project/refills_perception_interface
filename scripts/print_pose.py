import yaml

import rospy
from refills_perception_interface.tfwrapper import lookup_pose, init

rospy.init_node('pose_printer')
init(1, 1)
p = lookup_pose('map', 'base_footprint')
p.header.stamp.secs = 0.0
p.header.stamp.nsecs = 0.0
print(yaml.dump(str(p)).replace('\\"\\\n  ','\'').replace('\\\n  \\','').replace('\\n','\n').replace('\\"','\'').replace('"',''))