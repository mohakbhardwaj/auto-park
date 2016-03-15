#!/usr/bin/python


import rospy
import time
from std_msgs.msg import String

rospy.init_node("talker_xbee")
pub1 = rospy.Publisher("xbee_update", String, queue_size=2)

       
while not rospy.is_shutdown():
    data = raw_input('enter data to send to xbee update: ')
    pub1.publish(str(data))
pass
