#!/usr/bin/python

import rospy
from mabplanner.msg import rt_data
import numpy as np
from math import sqrt,log
from gplanner.srv import mab
import rospkg
import operator
from operator import itemgetter
from collections import defaultdict
import sys

rospy.init_node("MabPlanner")

segments_current = defaultdict(list) # {segmentid:[Q(a), n(a), numOfSpotsinArea, total_cost}
spots_data = defaultdict(list) # {spotId: [park_time, exit_time, areaId]} 
nStep = 0
vel = 5.55 #in m/s
costUpperBound = 206/vel

def id2areaId(spotId):
	row = (spotId)/52
	column = abs(int(((spotId - 1)%52)/6.5)%2 - 1)
	return column + 2*row

def init():
	global spots_data, segments_current
	rospack = rospkg.RosPack()
	cost_path = rospack.get_path('mabplanner') + "/src/costs.txt"
	f = open(cost_path, 'r')
	costlist = [float(line) for line in f]
	for i in xrange(len(costlist)):
		areaId = id2areaId(i)
		# spots_data[i] = [0.0, (costlist[i]/vel), areaId]
		spots_data[i] = [0.0, areaId*0.1, areaId]
	
	# normsum=0
	# for i in spots_data.values():
	# 	normsum+=i[1]

	# for i in spots_data.values():
	# 	i[1]=i[1]/float(normsum)

	numSpotsinArea = [0]*4
	initialCostinArea = [0]*4

	for i in spots_data.values():
		numSpotsinArea[i[2]] += 1
		initialCostinArea[i[2]] += i[0] + i[1]
	
	
	for j in xrange(len(numSpotsinArea)):
		segments_current[j] = [initialCostinArea[j]/numSpotsinArea[j], 1, numSpotsinArea[j], initialCostinArea[j]/numSpotsinArea[j]]

	# costUpperBound = max(segments_current.iteritems(), key = operator.itemgetter(1)[0])[0]



def populate(data):
	#if park or exit then update current cost else when car just enters update on nStep
	global spots_data, nStep
	if data.action == "park":
		spots_data[data.id-1][0] = data.time
		# updateStats(data.id,data.time)
		
	elif data.action == "return":
		spots_data[data.id-1][1] = data.time
		# updateStats(data.id,data.time)
	# else:
	# 	nStep = data.ctr
		print nStep

def mabArmPull(req):
	global nStep
	nStep+=1
	sums = [0]*4
	
	global segments_current	# {segmentid:[Q(a), n(a), numOfSpotsinArea, total_cost}

	#update Q for each area by iterating over all spots
	for j in spots_data.values(): # {spotId: [park_time, exit_time, areaId]}
		# sys.stdout.write("length of j is "+str(len(j)))
		# sys.stdout.write("Requested indice is "+ str(j[2]))
		try:
			segments_current[j[2]][0]+= (j[1]+j[2]) /(float(nStep)*segments_current[j[2]][2])
		except:
			sys.stdout.write("J is "+ str(j))
	#update Qtotal for each area
	for j in segments_current.values():
		j[3]=(j[0]/costUpperBound) - sqrt((2*log(nStep) )/j[1])

	bestArea=min(segments_current.iteritems(), key = lambda p: p[1][3])[0]
	
	segments_current[bestArea][1] += 1  #Update the fact that the area has been selected once
	
	print "Best Area Selected: ",bestArea
	return bestArea


def main():
    init()
    rospy.Subscriber("realtime_update", rt_data, populate)
    global_service = rospy.Service("segmentRequest", mab, mabArmPull)
    rospy.spin()




# def updateStats(spotId,time):
# 	global segments_current

# 	activeArea = spots_data[spotId][2]
	
# 	segments_current[activeArea][0] = 0.8*segments_current[activeArea][0]*(nStep-1) + (time/segments_current[activeArea][2])/nStep
# 	segments_current[activeArea][3] = (segments_current[activeArea][0]/costUpperBound) + sqrt((2*log(nStep) )/segments_current[activeArea][1])


		


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass