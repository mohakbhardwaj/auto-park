#!/usr/bin/python

import rospy
from std_msgs.msg import String, Int64
from collections import deque
import time
import oculusprimesocket

rospy.init_node("master_autopark")
blue_push = rospy.Publisher("blue_push", String, queue_size=5)
serial_push = rospy.Publisher("xbee_update", String, queue_size=2)
last_command = deque(maxlen=2) # only remember the last command and the current one
last_command.append("Idle") # initial state
oculusprimesocket.connect() # initiate connection to oculus prime
flag = 0 # 1 = obstacle, don't send commands
apptoxbee = {"2": "park", "4": "return"}

def action(data):
    global last_command, status
    #  if the last message is different from the current message received from app
    cmd = data.data
    if cmd != last_command[-1]: # if the last message is different from the current one
        last_command.append(cmd) # store command
        status = "ongoing" # not reached
        serial_push.publish(apptoxbee[cmd]) # send request for XBee
        oculusprimesocket.sendString("strobeflash on 1000 30") # on for 1000 ms at 30% intensity, indicates command was received
        time.sleep(10)
        # while status != "reached": # wait until oculus has reached destination
        status = rospy.wait_for_message("destination", String).data
        #    time.sleep(5)
        blue_push.publish(str(int(cmd) + 1)) # send 3 if parked, 5 if returned


def start():
    rospy.Subscriber("blue_pull", String, action)
    rospy.spin()

start()
