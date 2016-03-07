#!/usr/bin/python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher("spot_info", String, queue_size=10)



def callback(data):
	print data.data
if __name__ == '__main__':


	rospy.init_node('test')
	rospy.Subscriber("optimal_spot", String, callback)
	print "y or n"
	dec = raw_input()
	if dec == "y":
		pub.publish(dec)

	rospy.spin()
