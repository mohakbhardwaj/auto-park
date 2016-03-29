#!/usr/bin/python

import rospy
import time
from ros_assignment.msg import path_id
import numpy as np
from random import randint
from localplanner.srv import optimPath
from geometry_msgs.msg import PoseStamped
import tf
from gplanner.srv import OptimalSpotGenerator


rospy.init_node("Backend")
command = rospy.Publisher("simulation_backend", path_id, queue_size=19)
msg = path_id()
time_departure_off = 120

cars_arrival = np.array([3*abs(8 - round(elem, 1)) for elem in np.random.rayleigh(2, 108)])
cars_stay = np.array([time_departure_off + 3*round(elem, 1) for elem in np.random.chisquare(3, 108)])
cars_size = [randint(1, 3) for _ in range(0, 108)]
ids = np.array(xrange(0, 107))

time_init = time.time()

print "1"
rospy.wait_for_service('optimPath')
print "2"
rospy.wait_for_service('OptimalSpotGenerator')
print "3"
fetch_path = rospy.ServiceProxy('optimPath', optimPath)
fetch_spot = rospy.ServiceProxy('OptimalSpotGenerator', OptimalSpotGenerator)

parking_dict = {}

entrance = PoseStamped()
entrance.pose.position.x = 2.5
entrance.pose.position.y = 2
entrance.pose.position.z = 0
entrance.pose.orientation.x, entrance.pose.orientation.y, entrance.pose.orientation.z, entrance.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, 0)
exitc = PoseStamped()
exitc.pose.position.x = 40
exitc.pose.position.y = 46
exitc.pose.position.z = 0
exitc.pose.orientation.x, exitc.pose.orientation.y, exitc.pose.orientation.z, exitc.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, 0)

bogus = PoseStamped()
bogus.pose.position.x = 40
bogus.pose.position.y = 26
bogus.pose.position.z = 0
bogus.pose.orientation.x, bogus.pose.orientation.y, bogus.pose.orientation.z, bogus.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, 0)


while True:
    time.sleep(0.1)
    time_off = time.time() - time_init
    remove_index = []
    for i in range(0, len(cars_arrival)):
        if time_off > cars_arrival[i]:
            msg.state = "arrive"
            msg.id = ids[i]
            print "Publish a car"
            # spot = fetch_spot(True)
            # parking_dict[ids[i]] = spot.data
            resp = fetch_path([entrance, bogus])
            msg.result = resp.path
            command.publish(msg)
        elif time_off > cars_stay[i]:
            msg.state = "return"
            msg.id = ids[i]
            print "Remove a car"
            resp = fetch_path([bogus, exitc])
            msg.result = resp.path
            command.publish(msg)
            remove_index.append(i)
    ids = np.delete(ids, remove_index, None)
    cars_arrival = np.delete(cars_arrival, remove_index, None)
    cars_stay = np.delete(cars_stay, remove_index, None)
