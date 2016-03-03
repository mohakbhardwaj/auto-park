#!usr/env/bin/python

import rospy
import time
import numpy as np
from random import randint
from simulation.msg import *


rospy.init_node("Backend")
command = rospy.Publisher("simulation_backend", commands, queue_size=19)
msg = commands()
mean_time = 60
deviation_stay = 30
deviation_arrival = 20

cars_arrival_time = [round(elem, 1) for elem in np.random.normal(mean_time, deviation_arrival, 200)]
cars_arrival = np.array([sum(cars_arrival_time[0:off]) for off in range(1, len(cars_arrival_time))])
cars_stay_time = [round(elem, 1) for elem in np.random.normal(mean_time, deviation_stay, 200)]
cars_stay = np.array(np.sum([sum(cars_stay_time[0:off]) for off in range(1, len(cars_stay_time))], cars_arrival, 0))
cars_size = [randint(1, 3) for _ in range(0, 200)]
ids = np.array(xrange(0, 200))

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
