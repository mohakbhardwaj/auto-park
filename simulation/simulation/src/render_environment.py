#!/usr/bin/python

from __future__ import division
import rospy
import time
import tf
import numpy as np
import rospkg
import math
import pickle

from visualization_msgs.msg import *
from simulation.msg import path_id
from simulation.srv import local_request, global_request_render
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from random import random, randint
from mabplanner.msg import rt_data
from threading import Thread

rospy.init_node("Render")
rviz = rospy.Publisher("visualization_msgs", Marker, queue_size=0, latch=True)
mab = rospy.Publisher("realtime_update", rt_data, queue_size=0)


smoothness = 10
speed = 2 * smoothness
current_time = [0]
path_resolution = 5
parking_duration = {}
returning_duration = {}
pause_duration = {}
vehicles = []
physical_location = [[i, 99, 99] for i in range(0, 108)]
priority_count = 0
rospack = rospkg.RosPack()
stl_path = "file://" + rospack.get_path('simulation') + "/src/res/"
cars_dict = {1: "Models/Batmobile/Batmobile.dae", 2: "Models/Protect_Van/Protect_Van.dae",
             3: "Models/coupe_1/coupe.dae", 4: "Models/Jeep_rigged/Jeep_rigged.dae",
             5: "Models/Lincoln_rigged/Lincoln_rigged.dae"}
scale_dict = {1: [0.32, 0.32, 0.32], 2: [0.3, 0.3, 0.3], 3: [0.25, 0.25, 0.25], 4: [0.32, 0.32, 0.32],
              5: [0.32, 0.32, 0.32]}

with open(rospack.get_path('simulation') + "/src/id_to_config.p", "rb") as f:
    spots = pickle.load(f)

spots_id = list(spots)
spots_config = spots.values()


# class which takes care of all the aspects of a vehicle
class Car:
    def __init__(self, vehicle_id, path, priority, pd, pc):
        # set the markers associated with this vehicle
        self.priority = priority
        self.initiate = 0
        self.check = 0
        self.vehicle_marker = Marker()
        self.vehicle_marker.header.frame_id = 'map'
        self.vehicle_marker.action = Marker.ADD
        self.destination_marker = Marker()
        self.destination_marker.header.frame_id = 'map'
        self.path_marker = Marker()
        self.path_marker.header.frame_id = 'map'
        self.path_color = ColorRGBA()
        select_random = randint(2, 5)
        self.vehicle_marker.type = Marker.MESH_RESOURCE
        if vehicle_id != 1:
            model_path = stl_path + cars_dict[select_random]
        else:
            select_random = 1
            model_path = stl_path + cars_dict[select_random]
        self.vehicle_marker.mesh_resource = model_path
        self.vehicle_marker.mesh_use_embedded_materials = True
        self.destination_marker.type = Marker.CYLINDER
        self.path_marker.type = Marker.LINE_STRIP
        self.vehicle_marker.ns = "Vehicle"
        self.destination_marker.ns = "Markers"
        self.path_marker.ns = "Path"
        self.state = "arrive"
        self.vehicle_marker.color.a = 1
        self.destination_marker.color.a = 1
        self.path_color.a = 1
        self.color = [round(random(), 2) for _ in range(0, 3)]
        self.id = vehicle_id
        self.draw_path(path, self.state, pd, pc)
        self.motion_current = current_time
        self.motion = 1
        self.path_index = 0
        self.vehicle_marker.id = vehicle_id
        self.destination_marker.id = vehicle_id
        self.path_marker.id = vehicle_id
        self.vehicle_marker.color.r, self.vehicle_marker.color.g, self.vehicle_marker.color.b = self.color
        self.destination_marker.color.r, self.destination_marker.color.g, self.destination_marker.color.b = self.color
        self.path_color.r, self.path_color.g, self.path_color.b = self.color
        self.vehicle_marker.scale.x, self.vehicle_marker.scale.y, self.vehicle_marker.scale.z = scale_dict[select_random]
        self.destination_marker.scale.x, self.destination_marker.scale.y, self.destination_marker.scale.z = [2.5, 3.5, 0.1]
        self.path_marker.scale.x = 0.1
        self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = [
            0, 0, 0, 1]
        self.destination_marker.pose.orientation.x, self.destination_marker.pose.orientation.y, self.destination_marker.pose.orientation.z, self.destination_marker.pose.orientation.w = [
            0, 0, 0, 1]
        self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y, self.vehicle_marker.pose.position.z = [
            2.5, 2, 0]
        self.destination_marker.pose.position.z = 0
        self.spot_id = spots_id[spots_config.index(self.vehicle_path[-1][0:2])]
        self.initiate = 1

    def quatfromang(self, yaw):
        # get quaternion for euler angles
        return tf.transformations.quaternion_from_euler(0, 0, yaw + 1.57)

    def angfromquat(self, quat):
        # get euler angles for quaternion
        return tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

    def pause(self):
        self.motion_start = current_time[0] - (self.path_index)/ speed - 0.5

    def move(self):
        # update the state of the vehicle
        self.path_index = int((self.motion_current[0] - self.motion_start) * speed)
        if self.path_index >= len(self.interpolated_path):
            self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = self.quatfromang(
                    self.vehicle_path[-1][2])
            self.clear_path()
        else:
            self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y = self.interpolated_path[
                                                                                           self.path_index][0:2]
            self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = self.quatfromang(
                    self.interpolated_path[self.path_index][2])
            self.path_marker.points = self.points_trace[int(self.path_index / path_resolution):-1]
            self.path_marker.colors = self.color_trace[int(self.path_index / path_resolution):-1]

    def curve(self, start, curvature, distance):
        x, y, th = start
        tx = math.cos(x)
        ty = math.sin(x)
        radius = 1/curvature
        xc = x - radius*ty
        yc = y + radius*tx
        angle = distance/radius
        cosa = math.cos(angle)
        sina = math.sin(angle)
        nx = xc + radius*(cosa*ty + sina*tx)
        ny = yc + radius*(sina*ty - cosa*tx)
        nth = (th + angle + np.pi) % (2*np.pi) - np.pi
        return [nx, ny, nth]

    def interpolate(self):
        # add intermediate steps for smooth motion
        for j in range(0, len(self.vehicle_path) - 2):
            if self.vehicle_path[j][2] != self.vehicle_path[j + 1][2]:
                steps = int(self.distances[j+1] * smoothness)
                l = 0
                dl = math.copysign(self.distances[j+1]/steps, self.distances[j+1])
                while abs(l) < abs(self.distances[j+1]):
                    self.interpolated_path.append(self.curve(self.vehicle_path[j], self.curvatures[j+1], l))
                    l += dl
            else:
                steps = int(self.distances[j+1] * smoothness)
                xspan = [round(val, 2) for val in
                         np.linspace(self.vehicle_path[j][0], self.vehicle_path[j + 1][0], steps)]
                yspan = [round(val, 2) for val in
                         np.linspace(self.vehicle_path[j][1], self.vehicle_path[j + 1][1], steps)]
                heading = [self.vehicle_path[j][2]] * steps
                self.interpolated_path += zip(xspan[1:], yspan[1:], heading[1:])
        steps = int(np.linalg.norm(np.array(self.vehicle_path[-2][0:2]) - np.array(self.vehicle_path[-1][0:2]))*speed)
        xspan = [round(val, 2) for val in np.linspace(self.vehicle_path[-2][0], self.vehicle_path[-1][0], steps)]
        yspan = [round(val, 2) for val in np.linspace(self.vehicle_path[-2][1], self.vehicle_path[-1][1], steps)]
        heading = [round(val, 2) for val in np.linspace(self.vehicle_path[-2][2], self.vehicle_path[-1][2], steps)]
        self.interpolated_path += zip(xspan, yspan, heading)

    def draw_path(self, path, state, pd, pc):
        # draw the paths of the vehicle on RViz
        self.distances = pd
        self.curvatures = pc
        self.vehicle_path = []
        self.interpolated_path = []
        for i in range(0, len(path)):
            self.vehicle_path.append(
                    [path[i].pose.position.x, path[i].pose.position.y, self.angfromquat(path[i].pose.orientation)])
        self.destination_marker.action = Marker.ADD
        self.destination_marker.pose.position.x, self.destination_marker.pose.position.y = self.vehicle_path[-1][0:2]
        self.path_marker.action = Marker.ADD
        self.motion = 1
        self.motion_start = time.time()
        self.motion_init = time.time()
        self.interpolate()
        self.state = state
        pts = [0] * len(self.interpolated_path)
        for i in range(0, len(self.interpolated_path), path_resolution):
            pts[i] = Point()
            pts[i].z = 0
            pts[i].x = self.interpolated_path[i][0]
            pts[i].y = self.interpolated_path[i][1]
            self.path_marker.points.append(pts[i])
            self.path_marker.colors.append(self.path_color)
        pts.append(Point())
        pts[-1].z = 0
        pts[-1].x = self.interpolated_path[-1][0]
        pts[-1].y = self.interpolated_path[-1][1]
        self.path_marker.points.append(pts[i])
        self.path_marker.colors.append(self.path_color)
        self.points_trace = self.path_marker.points
        self.color_trace = self.path_marker.colors

    def clear_path(self):
        global parking_duration, pause_duration, returning_duration
        # clear the path of the vehicle
        if self.state == "arrive":
            parking_duration[self.id] = time.time() - self.motion_init
            pause_duration[self.id] = self.motion_start - self.motion_init
            self.state = "idle"
            mab_publish(self.spot_id, "park", parking_duration[self.id])
        elif self.state == "returning":
            returning_duration[self.id] = time.time() - self.motion_init
            pause_duration[self.id] += self.motion_start - self.motion_init
            self.clear()
            self.state = "clear"
            mab_publish(self.spot_id, "return", returning_duration[self.id])
        self.motion_start = 0
        self.motion_init = 0
        self.motion = 0
        self.vehicle_marker.pose.position.x, self.vehicle_marker.pose.position.y = self.vehicle_path[-1][0:2]
        self.vehicle_marker.pose.orientation.x, self.vehicle_marker.pose.orientation.y, self.vehicle_marker.pose.orientation.z, self.vehicle_marker.pose.orientation.w = self.quatfromang(self.vehicle_path[-1][2])
        self.path_marker.action = Marker.DELETE
        self.destination_marker.action = Marker.DELETE
        self.check = 0

    def clear(self):
        # publish a message to remove the vehicle
        self.vehicle_marker.action = Marker.DELETE

    def service_response(self):
        return self.interpolated_path[self.path_index]


def mab_publish(sid, action, t):
    temp = rt_data()
    temp.id = sid - 1
    temp.action = action
    temp.time = t
    temp.ctr = 0
    mab.publish(temp)


def vehicle_state(data):
    global vehicles, priority_count
    if data.state == "arrive":
        a = Car(data.id, data.result, priority_count, data.pd, data.pc)
        vehicles.append(a)
        priority_count += 1
        mab_ctr = rt_data()
        mab_ctr.id = 0
        mab_ctr.action = "counter"
        mab_ctr.time = 0
        mab_ctr.ctr = priority_count
        mab.publish(mab_ctr)
    elif data.state == "return":
        vehicles[data.id].draw_path(data.result, data.state, data.pd, data.pc)


def global_state(req):
    return {'spots_state': []}


def local_state(req):
    segments = [0] * 4
    location = [car.interpolated_path[car.path_index][0:2] for car in vehicles]
    for coords in location:
        if coords[0] > 21.25:
            if coords[1] > 24:
                segments[0] += 1
            else:
                segments[3] += 1
        else:
            if coords[1] > 24:
                segments[1] += 1
            else:
                segments[2] += 1
    return {'segments': segments, 'spots_state': []}


def update():
    rospy.Subscriber("simulation_backend", path_id, vehicle_state)
    global_service = rospy.Service("global_state_render", global_request_render, global_state)
    local_service = rospy.Service("local_state", local_request, local_state)
    rospy.spin()


def track():
    global physical_location
    for car in vehicles:
        if car.motion:
            physical_location[car.priority][1:3] = car.interpolated_path[car.path_index][0:2]
        elif car.check == 0:
            physical_location[car.priority][1:3] = [99, 99]
            car.check = 1


# have a custom message of line, cylinder marker, and a cube marker in each object of the class
# keep on updating the positions based on motion flag
def draw():
    global current_time, physical_location
    while True:
        time.sleep(0.0001)
        current_time[0] = time.time()
        track()
        for car in vehicles:
            if car.motion and car.initiate:
                flag = 0
                for i in range(0, car.priority):
                    if np.linalg.norm(np.array(physical_location[car.priority][1:3]) - np.array(
                            physical_location[i][1:3])) < 4:
                        car.pause()
                        flag = 1
                        break
                if flag == 0:
                    car.move()
                    rviz.publish(car.path_marker)
                    time.sleep(0.0001)
                    rviz.publish(car.vehicle_marker)
                    time.sleep(0.0001)
                    rviz.publish(car.destination_marker)
                    time.sleep(0.0001)


access = Thread(target=update)
render = Thread(target=draw)

access.start()
render.start()
