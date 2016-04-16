import rospy
from mabplanner.msg import rt_data
import numpy as np
from math import sqrt, log
from mabplanner.srv import optimArea
import rospkg

rospy.init_node("MabPlanner")

segments_current = {}
spots_data = {}
nStep = 0

def id2areaId(spotId):
	row = (spotId - 1)/52
	column = math.abs(int(((spotId - 1)%52)/6.5)%2 - 1)
	return column + 2*row

def init():
	global spots_data, segments_current
	rospack = rospkg.Rospack()
	cost_path = rospack.get_path('mabplanner') + "/src/costs.txt"
	f = open(cost_path, r)
	costlist = [float(line) for line in f]
	for i in xrange(len(costlist)):
		areaId = id2areaId(i)
		spots_data[i] = [0.0, costlist[i], areaId]
	for i in segments_current.values():
		numSpotsinArea[i[2]] += 1
		initialCostinArea[i[2]] += i[2]
	for j in xrange(len(numSpotsinArea)):
		segments_current[j] = [initialCostinArea[j]/numSpotsinArea[j], 0]



def populate(data):
	#if park or exit then update current cost else when car just enters update on nStep
	global spots_data, nStep
	if data.action == "park":
		spots_data[data.id][0] = data.time
		updateStats(data.id)
	elif data.action == "return":
		spots_data[data.id][1] = data.time
		updateStats(data.id)
	else:
		nStep = data.ctr


def mab(req):
	sums = [0]*4
	global segments_current
	# for i in spots_data.keys():
	# 	sums[spots_data[i][2]] += 
	a = []
	for segment in segments_current.keys():
		cost1 = segments_current[segment][0]
		nSelections = segments_current[segment][1]
		if nSelections != 0:
			cost2 = sqrt((2*log(req.nStep))/nSelections)
		else:
			cost2 = 0
		a.append(cost1 - cost2)
	
	bestArea = a.index(min(a))
	segments_current[bestArea][1] += 1  #Update the fact that the area has been selected once
	return bestArea


def main():
	rospy.Subscriber("realtime_updates", rt_data, populate)
	global_service = rospy.Service("segmentRequest", optimArea, mab)


def updateStats(spotId):
	global nStep
	# nStep = counter
	activeArea = spots_data[spotId][2]
	ctr = [0] * 4
	for i in segments_current.values():
		ctr[i[2]] += 1
	numSpotsinActiveArea = ctr[activeArea]
	segments_current[activeArea][0] = segments_current[activeArea][0]*nStep + ((spots_data[spotId][0] + spots_data[spotId][1])/numSpotsinActiveArea)



		


