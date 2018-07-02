
#!/usr/bin/env python

import roslib
#roslib.load_manifest('cool400_controller')

import time
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    pub = rospy.Publisher('gripper_controller/command', Float64);
    rospy.init_node('pose_gripper_close', anonymous=True)
    time.sleep(1)
    print "Closing gripper"
    pub.publish(-3.14)
