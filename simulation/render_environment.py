#!usr/env/bin/python

import rospy
import time
import random

from threading import Thread
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


# class which takes care of all the aspects of a vehicle
class Car:

    def __init__(self, vehicle_id, path):
        # set the color associated with this vehicle
        self.color_r = random.randint(0, 255)
        self.color_g = random.randint(0, 255)
        self.color_b = random.randint(0, 255)
        self.id = vehicle_id
        self.vehicle_path = path
        self.motion_start = 0
        self.motion = 0

    def move(self):
        # update the state of the vehicle
        self.motion_start = time.time()

    def draw_path(self, path):
        # draw the path of the vehicle on RViz
        self.vehicle_path = path
        self.motion = 1

    def clear_path(self):
        # clear the path of the vehicle
        self.vehicle_path = []
        self.motion_start = 0
        self.motion = 0

    def clear(self):
        # publish a message to remove the vehicle


vehicles = []

for i in range(0, 10):
    a = Car(i, [])
    vehicles.append(a)

# have a custom message of line, two markers, and a cube in each object of the class
# keep on updating the positions based on motion flag
