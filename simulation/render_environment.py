#!usr/env/bin/python

import rospy
import time
import random
import tf
import numpy as np

from visualization_msgs.msg import *
from simulation.msg import *


rviz = rospy.Publisher("visualization_msgs", Marker, queue_size=0, latch=True)
planner_data = rospy.Publisher("visualization_msgs", planner_transmit, queue_size=0, latch=True)

rospy.init_node("Render")
speed = 0.1


# class which takes care of all the aspects of a vehicle
class Car:

    def __init__(self, vehicle_id, path, size):
        # set the markers associated with this vehicle
        self.vehicle_marker = Marker()
        self.destination_marker = Marker()
        self.vehicle_marker.type = Marker.CUBE
        self.destination_marker.type = Marker.CYLINDER
        self.vehicle_marker.ns = "Vehicle"
        self.destination_marker.ns = "Markers"
        self.state = "idle"
        self.vehicle_marker.color.a = 1
        self.destination_marker.color.a = 1
        self.color = [round(random.random(), 2) for _ in range(0, 3)]
        self.id = vehicle_id
        self.vehicle_path = path
        self.motion_start = 0
        self.motion = 0
        self.path_index = 0
        self.interpolated_path = []
        self.vehicle_marker.id = vehicle_id
        self.destination_marker.id = vehicle_id
        self.vehicle_marker.color.r, self.vehicle_marker.color.g, self.vehicle_marker.color.b = self.color
        self.destination_marker.color.r, self.destination_marker.color.g, self.destination_marker.color.b = self.color
        self.vehicle_marker.scale.x, self.vehicle_marker.scale.y, self.vehicle_marker.scale.z = [size, 0.4, 0.5]
        self.destination_marker.scale.x, self.destination_marker.scale.y, self.destination_marker.scale.z = [size, size, 0.1]
        self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = [0, 0, 0, 1]
        self.destination_marker.pose.orientation.x, self.destination_marker.pose.orientation.y, self.destination_marker.pose.orientation.z, self.destination_marker.pose.orientation.w = [0, 0, 0, 1]
        self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y, self.vehicle_marker.pose.position.z = [0, 0, 0]
        self.destination_marker.pose.position.x, self.destination_marker.pose.position.y, self.destination_marker.pose.position.z = [0, 0, 0]

    def quatfromang(self, yaw):
        return tf.transformations.quaternion_from_euler(0, 0, yaw)

    def move(self):
        # update the state of the vehicle
        self.motion_start = time.time()

    def interpolate(self):
        for i in range(0, len(self.vehicle_path)):
            if self.vehicle_path[i][2] != self.vehicle_path[i+1][2]:
                self.interpolated_path += []
            else:
                steps = int(np.linalg.norm(np.array(self.vehicle_path[i+1][0:2]) - np.array(self.vehicle_path[i][0:2]))/speed)
                xspan = [round(val, 2) for val in np.linspace(self.vehicle_path[i][0], self.vehicle_path[i+1][0], steps)]
                yspan = [round(val, 2) for val in np.linspace(self.vehicle_path[i][1], self.vehicle_path[i+1][1], steps)]
                heading = self.vehicle_path[i][2] * steps
                self.interpolated_path += zip(xspan, yspan, heading)

    def draw_path(self, path):
        # draw the path of the vehicle on RViz
        self.vehicle_path = path
        self.motion = 1

    def clear_path(self):
        # clear the path of the vehicle
        self.vehicle_path = []
        self.interpolated_path = []
        self.motion_start = 0
        self.motion = 0
        self.destination_marker.action = Marker.DELETE

    def clear(self):
        # publish a message to remove the vehicle
        self.vehicle_marker.action = Marker.DELETE

    def pos_response(self):
        return self.interpolated_path[self.path_index]


vehicles = []

for ctr in range(0, 10):
    a = Car(ctr, [], 0.8)
    vehicles.append(a)

# have a custom message of line, two markers, and a cube in each object of the class
# keep on updating the positions based on motion flag
