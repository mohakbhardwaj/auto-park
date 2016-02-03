import rospy
from std_msgs.msg import String
import serial

ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
x = rospy.init_node("sup")

data = ""

while data == '':
    data = ser.readline()
    if data == "Parking":
        print "Car 1", data


def callback(data):
    stuff = data.data
    ser.write(stuff)


def subscribe():
    rospy.Subscriber("range_data", String, callback)
    rospy.spin()


subscribe()
