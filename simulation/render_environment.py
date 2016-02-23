#!usr/env/bin/python

import rospy
import time
import random

from threading import Thread
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

# class which takes care of all the aspects of a vehicle
class Car:


    def __init__(self, id):
        # set the color associated with this vehicle

    def move(self):
        # update the state of the vehicle


    def draw_path(self):
        # draw the path of the vehicle on RViz


    def clear_path(self):
        # clear the path of the vehicle
