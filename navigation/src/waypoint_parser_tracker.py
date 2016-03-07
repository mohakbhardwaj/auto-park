#!/usr/bin/env python

import oculusprimesocket
import time
import rospy
from locomotion_waypoint.msgs import waypoint
from std_msgs.msg import String
from threading import Thread

# intialize node, publisher and connect with Telnet
rospy.init_node("Locomotion")
pub1 = rospy.Publisher("platform_state", String, queue_size=2)
oculusprimesocket.connect()
motion = 0
stop = 0


# set the waypoint
def loco(data):
    global motion, wp, state, spot
    wp = data.goal
    state = data.obj
    spot = data.pos
    oculusprimesocket.sendString("gotowaypoint " + wp)
    motion = 1


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
    rospy.Subscriber("multi_agent_destination", waypoint, loco)
    rospy.Subscriber("master_control", Int64, emergency_stop)
    rospy.spin()


t1 = Thread(target=action)
t1.start()


# track the state of the platform
while True:
    while stop != 1 and motion == 1:  # loop
        time.sleep(1)  # wait one second, don't clog the telnet stream!
        oculusprimesocket.sendString("state roscurrentgoal")
        s = oculusprimesocket.waitForReplySearch("<state> roscurrentgoal")
        # check if roscurrentgoal is null (ie., deleted), means goal reached
        if s == "<state> roscurrentgoal null":
            print "waypoint " + wp + " reached"
            motion = 0
            if state == "exit":
                pub1.publish("returned")
            elif state == "park":
                # bunch of hardcoded commands to enter the spot
                if 0 < spot <= 6 or spot >= 18:
                    pass
                else:
                    pass
                pub1.publish("parked")
                pass
