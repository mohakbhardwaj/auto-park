#!usr/env/bin/python

import rospy
import time
from simulation.msg import *
import numpy as np
from random import randint

rospy.init_node("Backend")
command = rospy.Publisher("simulation_backend", commands, queue_size=19)
msg = commands()
time_departure_off = 120

cars_arrival = np.array([3*abs(8 - round(elem, 1)) for elem in np.random.rayleigh(2, 108)])
cars_stay = np.array([time_departure_off + 3*round(elem, 1) for elem in np.random.chisquare(3, 108)])
cars_size = [randint(1, 3) for _ in range(0, 108)]
ids = np.array(xrange(0, 107))

time_init = time.time()

while True:
    time.sleep(0.1)
    time_off = time.time() - time_init
    remove_index = []
    for i in range(0, len(cars_arrival)):
        if time_off > cars_arrival[i]:
            msg.state = "arrive"
            msg.id = ids[i]
            print "Publish a car"
            command.publish(msg)
        elif time_off > cars_stay[i]:
            msg.state = "return"
            msg.id = ids[i]
            print "Remove a car"
            command.publish(msg)
            remove_index.append(i)
    ids = np.delete(ids, remove_index, None)
    cars_arrival = np.delete(cars_arrival, remove_index, None)
    cars_stay = np.delete(cars_stay, remove_index, None)
