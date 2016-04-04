#!/usr/bin/python

import rospy
import time
from simulation.msg import path_id
import numpy as np
from localplanner.srv import optimPath
from geometry_msgs.msg import PoseStamped
import tf
from simulation.srv import global_request_backend
from gplanner.srv import OptimalSpotGenerator
from visualization_msgs.msg import *
from threading import Thread
from random import randint
import rosnode
from random import random

rviz = rospy.Publisher("visualization_msgs", Marker, queue_size=0, latch=True)
rospy.init_node("Backend")
command = rospy.Publisher("simulation_backend", path_id, queue_size=0)

x = 0

while x == 0:
    active_nodes = rosnode.get_node_names()
    x = len([sub for sub in active_nodes if 'rviz' in sub])
    time.sleep(0.1)

time.sleep(5)

msg = path_id()
time_departure_off = 120

number_of_vehicles = 30

scalingfactor = 6

cars_arrival = [scalingfactor*abs(8 - round(elem, 1)) for elem in np.random.rayleigh(2, number_of_vehicles)]
cars_stay = [time_departure_off + scalingfactor*round(elem, 1) for elem in np.random.chisquare(3, number_of_vehicles)]


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

time_init = time.time()

def draw_environment():
    boundary = Marker()
    boundary.type = Marker.CUBE
    boundary.header.frame_id = 'map'
    boundary.ns = "Boundary"
    boundary.color.a = 1
    boundary.color.r, boundary.color.g, boundary.color.b = [0.3, 0.3, 0.3]
    boundary.scale.y, boundary.scale.z = [0.1, 2]

    signs = Marker()
    signs.type = Marker.CUBE
    signs.header.frame_id = 'map'
    signs.ns = "Signs"
    signs.color.a = 0.8
    signs.pose.orientation.x, signs.pose.orientation.y, signs.pose.orientation.z, signs.pose.orientation.w = [0, 0, 0, 1]
    signs.scale.x, signs.scale.y, signs.scale.z = [5, 2.5, 0.1]

    positions = [[21.25, 0, 0], [42.5, 24, 0], [21.25, 48, 0], [0, 24, 0]]
    orientations = [[0, 0, 0, 1], [0, 0, 1, 1], [0, 0, 0, 1], [0, 0, 1, 1]]
    scale = [42.5, 48, 42.5, 48]
    position_sign = [[2.5, 1.25, 0.1], [40, 46.75, 0.1]]
    colors = [[0, 0.8, 0], [0.8, 0, 0]]

    tt = Marker()
    tt.header.frame_id = 'map'
    tt.type = Marker.TEXT_VIEW_FACING
    tt.scale.z = 10
    tt.action = Marker.ADD
    tt.ns = "WorldTime"
    tt.id = 1
    tt.color.a = 1
    tt.pose.position.x, tt.pose.position.y = [-10, 35]

    while True:
        tt.text = str(int(time.time() - time_init))
        rviz.publish(tt)
        if random() > 0.5:
            for i in range(4):
                boundary.id = i
                boundary.pose.orientation.x, boundary.pose.orientation.y, boundary.pose.orientation.z, boundary.pose.orientation.w = orientations[i]
                boundary.pose.position.x, boundary.pose.position.y, boundary.pose.position.z = positions[i]
                boundary.scale.x = scale[i]
                rviz.publish(boundary)
                time.sleep(0.1)

            for i in range(2):
                signs.id = i
                signs.pose.position.x, signs.pose.position.y, signs.pose.position.z = position_sign[i]
                signs.color.r, signs.color.g, signs.color.b = colors[i]
                rviz.publish(signs)
                time.sleep(0.1)


def global_state(req):
    time_queue = 2
    queue = 0
    for i in cars_arrival:
	if time_off + time_queue > i:
	    queue += 1
    return {'queue' : queue}


def response():
    global_service = rospy.Service("global_state_backend", global_request_backend, global_state)
    rospy.spin()


timer = Thread(target=draw_environment)
services = Thread(target=response)
timer.start()
services.start()

while True:
    time.sleep(0.5)
    time_off = time.time() - time_init
    for i in range(0, len(cars_arrival)):
        if time_off > cars_arrival[i]:
            cars_arrival[i] = 99999
            msg.state = "arrive"
            msg.id = i
            print "Publish car #", i
            # spot = fetch_spot(True)
            # parking_dict[ids[i]] = spot.spots
            bogus.pose.position.x = randint(30, 40)
            bogus.pose.position.y = randint(10, 40)
            resp = fetch_path([entrance, bogus])
            msg.result = resp.path
            command.publish(msg)
            time.sleep(5)
        """
        elif time_off > cars_stay[i]:
            cars_stay[i] = 99999
            msg.state = "return"
            msg.id = i
            print "Remove car#", i
            resp = fetch_path([bogus, exitc])
            msg.result = resp.path
            command.publish(msg)
        """
