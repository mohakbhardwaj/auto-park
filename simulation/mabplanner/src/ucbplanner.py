import rospy
from mabplanner.msg import rt_data
import numpy as np
from math import sqrt, log
from gplanner.srv import mab
import rospkg
import operator
from collections import defaultdict

rospy.init_node("MabPlanner")

segments_current = defaultdict(list) # {segmentid:[Q(a), n(a), numOfSpotsinArea, total_cost}
spots_data = defaultdict(list) # {spotId: [park_time, exit_time, areaId]} 
nStep = 0
vel = 5.55 #in m/s
costUpperBound = 0

def id2areaId(spotId):
	row = (spotId - 1)/52
	column = math.abs(int(((spotId - 1)%52)/6.5)%2 - 1)
	return column + 2*row

def init():
	global spots_data, segments_current, costUpperBound
	rospack = rospkg.Rospack()
	cost_path = rospack.get_path('mabplanner') + "/src/costs.txt"
	f = open(cost_path, r)
	costlist = [float(line) for line in f]
	for i in xrange(len(costlist)):
		areaId = id2areaId(i)
		spots_data[i] = [0.0, (costlist[i]/vel), areaId]
	
	numSpotsinArea = [0]*4
	initialCostinArea = [0]*4

	for i in spots_data.values():
		numSpotsinArea[i[2]] += 1
		initialCostinArea[i[2]] += i[0] + i[1]
	
	
	for j in xrange(len(numSpotsinArea)):
		segments_current[j] = [initialCostinArea[j]/numSpotsinArea[j], 0, numSpotsinArea[j], initialCostinArea[j]/numSpotsinArea[j]]

	costUpperBound = max(segments_current.iteritems(), key = operator.itemgetter(1)[0])[0]



def populate(data):
	#if park or exit then update current cost else when car just enters update on nStep
	global spots_data, nStep
	if data.action == "park":
		spots_data[data.id][0] = data.time
	elif data.action == "return":
		spots_data[data.id][1] = data.time
	updateStats(data.id,data.time)
	else:
		nStep = data.ctr


def mabArmPul(req):
	sums = [0]*4
	global segments_current
	bestArea = min(segments_current.iteritems(), key = operator.itemgetter(1)[3])[0]
	segments_current[bestArea][1] += 1  #Update the fact that the area has been selected once
	return bestArea


def main():
	rospy.Subscriber("realtime_updates", rt_data, populate)
	global_service = rospy.Service("segmentRequest", mab, mabArmPull)



def updateStats(spotId,time):
	global segments_current

	activeArea = spots_data[spotId][2]
	
	segments_current[activeArea][0] = (segments_current[activeArea][0]*(nStep-1) + (time/segments_current[activeArea][2])/nStep
	
	segments_current[activeArea][3] = (segments_current[activeArea][0]/costUpperBound) + sqrt((2*log(nStep))/segments_current[activeArea][1])


		


if __name__ == '__main__':
	main()