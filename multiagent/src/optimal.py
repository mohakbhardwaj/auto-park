#!/usr/bin/python
import sys
import numpy as np 
from heapq import heappush, heappop, heapify
import rospy
from std_msgs.msg import String
import time

pub1 = rospy.Publisher('destination', String, queue_size=10, latch=True)
pub2= rospy.Publisher('opt_spot', String, queue_size=10)



def callback(data):
	#input = data.data
	r = rospy.Rate(10)
	print data.data
	input = data.data[1:len(data.data)-1]
	input = input.split(", ")
        assumed_input = [int(x) for x in input]#[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
	print "Input is :", assumed_input



	"""val_spot = [(5, (2, 3), 0), (7, (2, 5), 1), (9, (2, 7), 2), (11, (2, 9), 3), 
			(13, (2, 11), 4), (10, (7, 3), 12), (9, (4, 3), 6), 
			(11, (4, 5), 7), (13, (4, 7), 8), (15, (4, 9), 9), (17, (4, 11), 10), 
			(19, (4, 13), 11), (15, (2, 13), 5), (12, (7, 5), 13), (14, (7, 7), 14), 
			(16, (7, 9), 15), (18, (7, 11), 16), (20, (7, 13), 17), (14, (9, 3), 18), 
			(16, (9, 5), 19), (18, (9, 7), 20), (20, (9, 9), 21), (22, (9, 11), 22), 
			(24, (9, 13), 23)]"""
	val_spot = [(5, (2, 3), 0),  (6, (2, 4), 1), (7, (2, 5), 2), (8, (2, 6), 3), 
				(9, (2, 7), 4) , (10, (2, 8), 5), (9, (4, 3), 6), (10, (4, 4), 7), 
				(11, (4, 5), 8), (12, (4, 6), 9), (13, (4, 7), 10), (14, (4, 8), 11), 
	 			(10, (7, 3), 12),(11, (7, 4), 13), (12, (7, 5), 14), (13, (7, 6), 15), 
	            (14, (7, 7), 16) ,(15, (7, 8), 17), (14, (9, 3), 18), (15, (9, 4), 19),
	  			 (16, (9, 5), 20), (17, (9, 6), 21), (18, (9, 7), 22), (19, (9, 8), 23)]


	heapify(val_spot)
	found = False
	fail = False
	while found == False and fail == False:
		curr_spot = heappop(val_spot)

		spot_val, spot_coordinates, input_idx = curr_spot


		if assumed_input[input_idx] == 0:
			optim_spot = curr_spot
			found = True
			print "Most Optial Spot Coordinates are :",  optim_spot[1]
		if len(val_spot) == 0:
			print "Failed"
			fail = True
	time.sleep(1)
	#pub1.publish(repr(optim_spot[1][0]) + " "+repr(optim_spot[1][1]))
        print optim_spot[2]+1
        pub1.publish(repr(optim_spot[2]+1))
        pub2.publish(repr(optim_spot[2]+1))
	r.sleep()
		
if __name__ == '__main__':

	time.sleep(5)
	print "Done sleeping"
	rospy.init_node('multi_agent')
	rospy.Subscriber("bs", String, callback)
	rospy.spin()
