#!/usr/bin/python


import rospy
import time
from std_msgs.msg import String

rospy.init_node("talker")
pub1 = rospy.Publisher("blue_push", String, queue_size=2)

       
while not rospy.is_shutdown():
    data = raw_input('enter data to send to app: ')
    pub1.publish(data)
pass
