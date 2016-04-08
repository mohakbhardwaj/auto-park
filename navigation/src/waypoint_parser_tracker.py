#!/usr/bin/env python

import oculusprimesocket
import time
import rospy
from std_msgs.msg import String, Int64
from threading import Thread

# intialize node, publisher and connect with Telnet
rospy.init_node("Locomotion")
pub1 = rospy.Publisher("platform_state", String, queue_size=2)
oculusprimesocket.connect()
motion = 0
stop = 0
time.sleep(5)
final = 0
prev = -1
cont = 0

# set the waypoint
def loco(data):
    global motion, wp, final, prev
    print "The waypoint is ", data.data
    wp = data.data
    if wp == "25":
	final = 0
	motion = 1
	oculusprimesocket.sendString("gotowaypoint " + str(prev) + "e")
	while motion == 1:
	    time.sleep(1)
	motion = 1
	final = 1
    	oculusprimesocket.sendString("gotowaypoint " + wp)
    	motion = 1
	final = 1
    else:
	prev = int(wp)
	transit = (int(wp) - 1)/6 + 1
	final = 0
	motion = 1
	oculusprimesocket.sendString("gotowaypoint transit" + str(transit))
	while motion == 1:
	    time.sleep(1)
	motion = 1
	if (int(wp) - 1)%6 < 3:
	    oculusprimesocket.sendString("gotowaypoint " + str((transit - 1)*6 + 3) + "e")
	while motion == 1:
	    time.sleep(1)
	motion = 1
	oculusprimesocket.sendString("gotowaypoint " + wp + "e")
	while motion == 1:
	    time.sleep(1)
	final = 1
	oculusprimesocket.sendString("gotowaypoint " + wp)

# process the incoming commands from the master and control the state of the platform
def emergency_stop(data):
    global stop
    if data.data == 0:  # 0 means clear e-stop
        stop = 0
        if motion == 1:
            oculusprimesocket.sendString("gotowaypoint " + wp)
    elif data.data == 1:  # 1 means raise e-stop
        stop = 1
        if motion == 1:
            oculusprimesocket.sendString("rosgoalcancel TRUE")


# start the Subscriber threads
def action():
    rospy.Subscriber("destination", String, loco)
    rospy.Subscriber("master_control", Int64, emergency_stop)
    rospy.spin()


t1 = Thread(target=action)
t1.start()


# track the state of the platform
while True:
    while stop != 1:  # loop
        time.sleep(1)  # wait one second, don't clog the telnet stream!
        s = oculusprimesocket.waitForReplySearch("navigation").lower().split()[-1]
        if s == "reached":
            print "waypoint " + wp + " reached"
            motion = 0
	    if final == 1:
                pub1.publish("reached")

