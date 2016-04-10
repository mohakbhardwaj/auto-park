#!/usr/bin/python

import rospy
import time
import ast
from std_msgs.msg import String
from autopark.msg import opt_choices, plan_msg


rospy.init_node("planner", anonymous=True)
pub1 = rospy.Publisher("opt", opt_choices, queue_size = 10)


def listToWaypoints(wylist):
    wypts = {}
    if len(wylist) > 1:
        num_wypts = wylist[0]
        print ("num_wypts: " + str(num_wypts))
        print ("len wylist: " + str(len(wylist)))
        del wylist[0]
        for i in range(num_wypts):
            num_coords = wylist[0]
            del wylist[0]
            temp = []
            for i in range(num_coords):
                temp.append([wylist[1], wylist[2]])
                del wylist[1:3]
            wypts[wylist[0]] = temp
            del wylist[0]

    return wypts


def callback(data):
    print "received data"
    curr_spot = data.curr_spot
    spot_options = ast.literal_eval(data.spot_options)
    print ("data wpts: " + str(ast.literal_eval(data.wpts)))
    waypoints = listToWaypoints(ast.literal_eval(data.wpts)).values() # don't care about vehicle IDs
    print ("current spot: " + str(curr_spot) + ", spot options: " + str(spot_options) + ", waypoints: " + str(waypoints))

    msg = opt_choices()
    if len(spot_options) > 1:
        msg.opt_spot = int(spot_options[int(len(spot_options)/2)])
    else:
        msg.opt_spot = 25
    msg.wpts = str([[0,0],[0,3],[5,3],[5,9]])
    print ("opt spot: " + str(msg.opt_spot) + ", waypoints to spot: " + str(msg.wpts))
    time.sleep(3)
    pub1.publish(msg)

def listener():
    time.sleep(3)
    print "starting planner substitute"
    rospy.Subscriber("planner", plan_msg, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()