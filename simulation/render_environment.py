#!usr/env/bin/python

import rospy
import time
import random
import tf
import numpy as np
from threading import Thread

from visualization_msgs.msg import *
from simulation.msg import *
from geometry_msgs.msg import Point


rviz = rospy.Publisher("visualization_msgs", Marker, queue_size=0, latch=True)
planner_data = rospy.Publisher("active_state", sim_transmit, queue_size=0, latch=True)

rospy.init_node("Render")
speed = 0.1
current_time = [0]
path_resolution = 5
parking_times = []
returning_times = []
vehicles = []


# class which takes care of all the aspects of a vehicle
class Car:

    def __init__(self, vehicle_id, path, size):
        # set the markers associated with this vehicle
        self.vehicle_marker = Marker()
        self.destination_marker = Marker()
        self.path_marker = Marker()
        self.vehicle_marker.type = Marker.CUBE
        self.destination_marker.type = Marker.CYLINDER
        self.path_marker.type = Marker.LINE_STRIP
        self.vehicle_marker.ns = "Vehicle"
        self.destination_marker.ns = "Markers"
        self.path_marker.ns = "Path"
        self.state = "idle"
        self.vehicle_marker.color.a = 1
        self.destination_marker.color.a = 1
        self.path_marker.color.a = 1
        self.color = [round(random.random(), 2) for _ in range(0, 3)]
        self.id = vehicle_id
        self.vehicle_path = path
        self.motion_start = 0
        self.motion_current = current_time
        self.motion = 0
        self.path_index = 0
        self.interpolated_path = []
        self.vehicle_marker.id = vehicle_id
        self.destination_marker.id = vehicle_id
        self.path_marker = vehicle_id
        self.vehicle_marker.color.r, self.vehicle_marker.color.g, self.vehicle_marker.color.b = self.color
        self.destination_marker.color.r, self.destination_marker.color.g, self.destination_marker.color.b = self.color
        self.path_marker.color.r, self.path_marker.color.g, self.path_marker.color.b = self.color
        self.vehicle_marker.scale.x, self.vehicle_marker.scale.y, self.vehicle_marker.scale.z = [size, 0.4, 0.5]
        self.destination_marker.scale.x, self.destination_marker.scale.y, self.destination_marker.scale.z = [size, size, 0.1]
        self.destination_marker.scale.x = 1
        self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = [0, 0, 0, 1]
        self.destination_marker.pose.orientation.x, self.destination_marker.pose.orientation.y, self.destination_marker.pose.orientation.z, self.destination_marker.pose.orientation.w = [0, 0, 0, 1]
        self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y, self.vehicle_marker.pose.position.z = [0, 0, 0]
        self.destination_marker.pose.position.x, self.destination_marker.pose.position.y, self.destination_marker.pose.position.z = [0, 0, 0]

    def quatfromang(self, yaw):
        # get quaternion for euler angles
        return tf.transformations.quaternion_from_euler(0, 0, yaw)

    def move(self):
        # update the state of the vehicle
        self.path_index = int((self.motion_current[0] - self.motion_start)/0.1)
        if self.path_index >= len(self.interpolated_path):
            self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y = self.interpolated_path[-1][0:2]
            self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = self.quatfromang(self.interpolated_path[-1][2])
            self.clear_path()
        else:
            self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y = self.interpolated_path[self.path_index][0:2]
            self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = self.quatfromang(self.interpolated_path[self.path_index][2])
            self.vehicle_path.points = self.vehicle_path.points[int(self.path_index/path_resolution):-1]

    def interpolate(self):
        # add intermediate steps for smooth motion
        for i in range(0, len(self.vehicle_path)):
            if self.vehicle_path[i][2] != self.vehicle_path[i+1][2]:
                self.interpolated_path += []
            else:
                steps = int(np.linalg.norm(np.array(self.vehicle_path[i+1][0:2]) - np.array(self.vehicle_path[i][0:2]))/speed)
                xspan = [round(val, 2) for val in np.linspace(self.vehicle_path[i][0], self.vehicle_path[i+1][0], steps)]
                yspan = [round(val, 2) for val in np.linspace(self.vehicle_path[i][1], self.vehicle_path[i+1][1], steps)]
                heading = self.vehicle_path[i][2] * steps
                self.interpolated_path += zip(xspan, yspan, heading)

    def draw_path(self, path, state):
        # draw the path of the vehicle on RViz
        self.vehicle_path = path
        self.motion = 1
        self.motion_start = time.time()
        self.interpolate()
        self.state = state
        pts = Point()
        pts.z = 0
        for i in range(0, len(self.interpolated_path), path_resolution):
            pts.x = self.interpolated_path[i][0]
            pts.y = self.interpolated_path[i][1]
            self.vehicle_path.points.append(pts)

    def clear_path(self):
        # clear the path of the vehicle
        if self.state == "parking":
            parking_times.append(len(self.interpolated_path)/speed)
            self.state = "idle"
        elif self.state == "returning":
            returning_times.append(len(self.interpolated_path)/speed)
            self.state = "idle"
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


def callback(data):
    if data.flag == "ADD":
        a = Car(data.id, data.path, data.size)
        vehicles.append(a)
    elif data.flag == "UPDATE":
        vehicles[data.id].draw_path(data.path)


def update():
    rospy.Subscriber("render_push", vehicle_update, callback)
    rospy.spin()


# have a custom message of line, cylinder marker, and a cube marker in each object of the class
# keep on updating the positions based on motion flag
def draw():
    global current_time
    while True:
        current_time[0] = time.time()
        for car in vehicles:
            if car.motion:
                car.move()

access = Thread(target=update)
render = Thread(target=draw)

access.start()
render.start()
