import rospy
from std_msgs.msg import String

x = rospy.init_node("sup")


def callback(data):
    print "Obstacle Detected"
    print data.data


def subscribe():
    rospy.Subscriber("range_data", String, callback)
    rospy.spin()


subscribe()