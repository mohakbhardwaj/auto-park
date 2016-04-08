#!/usr/bin/env python

import rospy
import serial
import subprocess
import time
import os
import signal
import numpy as np
import hashlib
import sys
from std_msgs.msg import String
from autopark.msg import list_ui
from threading import Thread
from heapq import heappush, heappop, heapify
from protocolwrapper import (
    ProtocolWrapper, ProtocolStatus)
from myformat import (
    message_format, message_crc, Container)

PROTOCOL_HEADER = '$'
PROTOCOL_FOOTER = '$'
PROTOCOL_DLE = '\x90'

global hellocount
global vcl_id
global liq_id
vcl_dict = {}
hello_dict = {}
vcl_spot = 0 #receive from planner

if len(sys.argv) > 1:
    if int(sys.argv[1]) == 1:
        isUI = True
    else:
	isUI = False
else:
    isUI = False

rospy.init_node("Xbee")
pub1 = rospy.Publisher("ui_update", list_ui, queue_size = 10)
pub2 = rospy.Publisher("destination", String, queue_size = 10, latch = True) # exit coordinates to navigation


# send GOODBYE to other XBees when ctrl+c or ctrl+z heard, then shut down
def signal_handler(signal, frame):
    print("I sent GOODBYE")
    mymsg = build_message_to_send(vcl_id,'GOODBYE',{0:0})
    ser.write(mymsg)
    if isUI:
        sendUIUpdate('returned', vcl_id, 25)
    time.sleep(1)
    os.kill(os.getpid(), 9)


# define ctrl+c and ctrl+z as signals to quit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTSTP, signal_handler)

# connects to other XBees
def connect():
    global ser, flag
    x = subprocess.check_output("ls /dev/serial/by-id", shell = True).split()
    if x[0].find("Digi") != -1:
        y = subprocess.check_output("dmesg | grep tty", shell = True).split()
        indices = [i for i, x in enumerate(y) if x == "FTDI"]
        address = "/dev/" + y[indices[-1]+8]
        ser = serial.Serial(address, 9600, timeout = 1)
        ser.flushInput()
        ser.flushOutput()
        flag = 0
    else:
        raise ValueError
    return


# builds message to send to XBees
def build_message_to_send(vcl_id, message_type, vcl_dict):
    data = dictToList(vcl_dict)
    datalen = len(data)
  
    raw = message_format.build(Container(
        vcl_id = vcl_id,      
        message_type = message_type,
        datalen = datalen,
        data = data,
        crc = 0))
       
    msg_without_crc = raw[:-8]    
    msg_crc = message_crc.build(Container(
        crc = int(''.join([i for i in hashlib.md5(msg_without_crc).hexdigest() if i.isdigit()])[0:10])))
   
    msg = msg_without_crc + msg_crc
    
    pw = ProtocolWrapper(
            header = PROTOCOL_HEADER,
            footer = PROTOCOL_FOOTER,
            dle = PROTOCOL_DLE)
    return pw.wrap(msg)


# parses message from XBees
def recover_msg(msg):
        pw = ProtocolWrapper(
        header = PROTOCOL_HEADER,
        footer = PROTOCOL_FOOTER,
        dle = PROTOCOL_DLE)

        status = map(pw.input, msg)
        rec_crc = 0
        calc_crc = 1
        
        if status[-1] == ProtocolStatus.MSG_OK:
                rec_msg = pw.last_message
                rec_crc = message_crc.parse(rec_msg[-8:]).crc
                calc_crc = int(''.join([i for i in hashlib.md5(rec_msg[:-8]).hexdigest() if i.isdigit()])[0:10])

        if rec_crc != calc_crc:
            print 'Error: CRC mismatch'

        return rec_msg


# reads message from XBees, parses, and returns it
def getmsg():
    time.sleep(0.5) 
    getmsg = ser.read(ser.inWaiting())
    rec_msg = recover_msg(getmsg)    
    rec_msg = message_format.parse(rec_msg)
    rec_msg.data = listToDict(rec_msg.data)
    return rec_msg


# Returns int list of ints in vcl_dict
def dictToList(vcl_dict):
    data = []
    for key, val in vcl_dict.iteritems():
        data.append(key)
        data.append(val)
    return data


# Returns integer list of int values in data
def listToDict(data):
    vcl_dict = {}
    vcl_dict = {data[i]: data[i+1] for i in range(0, len(data), 2)}
    return vcl_dict

# sends update to UI
def sendUIUpdate(stat, vcl, spot):
    msg = list_ui()
    msg.status = stat
    msg.vcl_id = vcl
    msg.spot_id = spot
    pub1.publish(msg)


# creates listeners to master, planner, and UI
def listen():
    rospy.Subscriber("xbee_update", String, callback_master)
    #rospy.Subscriber("opt_spot", String, callback_planner)
    rospy.Subscriber("vv_update", String, callback_UI)
    rospy.spin()


# messages from app or ...locomotion?
def callback_master(data):
    global vcl_dict,isUI,vcl_spot
    print "heard something from master:"
    print data.data
    
    # park master
    if data.data == 'park':
        print "heard park"

        bs = [0]*24
        print bs
        for val in vcl_dict.values():
            print val
            if val!= 25 and val!= 0:
                bs[val-1] = 1

        print "the bs:"
        print bs

        #start planner

        val_spot = [(5, (2, 3), 0),  (6, (2, 4), 1), (7, (2, 5), 2), (8, (2, 6), 3), (9, (2, 7), 4) , (10, (2, 8), 5), (9, (4, 3), 6), (10, (4, 4), 7), (11, (4, 5), 8), (12, (4, 6), 9), (13, (4, 7), 10), (14, (4, 8), 11), (10, (7, 3), 12),(11, (7, 4), 13), (12, (7, 5), 14), (13, (7, 6), 15), (14, (7, 7), 16) ,(15, (7, 8), 17), (14, (9, 3), 18), (15, (9, 4), 19), (16, (9, 5), 20), (17, (9, 6), 21), (18, (9, 7), 22), (19, (9, 8), 23)]


        heapify(val_spot)
        found = False
        fail = False
        while found == False and fail == False:
                curr_spot = heappop(val_spot)

                spot_val, spot_coordinates, input_idx = curr_spot

                if bs[input_idx] == 0:
                        optim_spot = curr_spot
                        found = True
                        print "Most Optimal Spot Coordinates are :",  optim_spot[1]
                if len(val_spot) == 0:
                        print "Failed"
                        fail = True
        time.sleep(1)
        # destination coordinates
        #print 'dest coordinates'
        #print optim_spot[1][0] + " "+optim_spot[1][1]
        #pub2.publish(repr(optim_spot[1][0]) + " "+repr(optim_spot[1][1]))
        print 'opt spot ID'
        print optim_spot[2]+1
        print pub2.publish(repr(optim_spot[2]+1))

        #end planner

        #start callback

        print "original dict"
        print vcl_dict

        vcl_spot = optim_spot[2]+1
        vcl_dict[vcl_id] = vcl_spot

        print "updated dict"
        print vcl_dict

        if isUI:
            sendUIUpdate('parking', vcl_id, vcl_spot)
            print 'sending selected spot to UI'

        # send update to other XBees
        print "Sending UPDATE because planner spot was selected and received"
        mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
        ser.write(mymsg)

        # end callback

    # return master
    elif data.data == 'return':
        print "heard return"
        print "sending exit ID (25) to nav"
        pub2.publish('25')

        vcl_dict[vcl_id] = 25
        vcl_spot = 25
        if isUI:
            print "sending returning to UI"
            sendUIUpdate('returning', vcl_id, vcl_spot)
        else:
            print "Sending UPDATE message b/c ROS msg received"
            mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
            ser.write(mymsg)

    # parked master
    elif data.data == 'parked':
        print "heard parked"

        if isUI:
            print "sending parked to UI"
            sendUIUpdate('parked', vcl_id, vcl_spot)
        else:
            print "Sending PARKED message b/c ROS parked received"
            mymsg = build_message_to_send(vcl_id, 'PARKED', {vcl_id:vcl_spot})
            ser.write(mymsg)

    # returned master
    elif data.data == 'returned':
        print "heard returned"

        if isUI:
            print "sending returned to UI"
            sendUIUpdate('returned', vcl_id, 25)
        else:
            print "Sending GOODBYE message b/c ROS returned received"
            mymsg = build_message_to_send(vcl_id, 'GOODBYE', {0:0})
            ser.write(mymsg)
        print "Shutting down"
        os.kill(os.getpid(), 9)
    
    else:
        print "Error: received unrecognized data from Master"


# when msg received from UI, update local dict and send update to other XBees
# Virtual vehicles
def callback_UI(data):
    global vcl_dict, vcl_id

    print "heard something from UI:"
    print data.data

    temp = data.data
    temp = temp.split(',')
    temp[0] = int(temp[0])
    temp[1] = int(temp[1])

    print "key and val:"
    print temp[0]
    print temp[1]
    print "original dict"
    print vcl_dict

    if temp[0] in vcl_dict:
        old_spot = vcl_dict[temp[0]]
    else:
        old_spot = temp[1]
    vcl_dict[temp[0]] = temp[1]

    print "updated dict"
    print vcl_dict

    if old_spot == 25 and temp[1] == 0:
        print "Sending GOODBYE b/c UI VV 25 -> 0"
        mymsg = build_message_to_send(temp[0], 'GOODBYE', {0:0})
    else:
        print "Sending UPDATE b/c UI VV message received"
        mymsg = build_message_to_send(temp[0], 'UPDATE', {temp[0]:temp[1]})

    ser.write(mymsg)


# messages from other XBees
def callback():
    print "entering callback"
    global liq, vcl_dict, liq_id, isUI, vcl_spot, hello_dict, vcl_id, hellocount

    while True:
        while ser.inWaiting()>0:
            print "received message"
            rec_msg = getmsg()
        
            # INTRO XBEE
            if rec_msg.message_type == 'INTRO':
                print "I received INTRO from ", rec_msg.vcl_id
                # if somehow two vehicles got the same ID OR
                # if intro vcl has id greater than that in local dict
                # then bump up person ID, replace dict and send update
                if rec_msg.vcl_id == vcl_id or rec_msg.vcl_id > liq_id:
                    print 'rec_msg.data'
                    print rec_msg.data
                    for key, val in rec_msg.data.iteritems():
                        vcl_dict[key] = val
                    liq_id = max(vcl_dict.keys())
                    vcl_id = liq_id + 1
                    vcl_dict[vcl_id] = vcl_spot
                    liq_id = vcl_id
                    print "I'm the liq"
                    # send update with personal spot
                    # vcl_id, msg_type, data
                    print "Sending update because erroneous INTRO received"
                    mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
                    ser.write(mymsg)
                if vcl_id == 0:
                    vcl_id = max(rec_msg.data.keys())+1
                    vcl_dict = rec_msg.data
                    vcl_dict[vcl_id] = vcl_spot
                    liq_id = vcl_id
                    print "I'm the liq"
                    print "Sending update because I have arrived"
                    mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
                    ser.write(mymsg)
                    #Tell the UI that I, and all my friends, exist.
                    if isUI:
                        for key, val in vcl_dict.iteritems():
                            msg = list_ui()
                            if val == 0:
                                sendUIUpdate('in_queue', key, val)
                            elif val == 25:
                                sendUIUpdate('returning', key, val)
                            else:
                                sendUIUpdate('parked', key, val)

            # HELLO XBEE
            if rec_msg.message_type == 'HELLO':
                print "I received HELLO from ", rec_msg.vcl_id

                # add liq_id+1 to hellos or increment val if necessary
                if liq_id+1 in hello_dict:
                    hello_dict[liq_id+1] = hello_dict[liq_id+1]+1
                else:
                    hello_dict[liq_id+1] = 1
    
                # find second to liq_id.  if len(dict) < 2, stliq_id is the liq_id
                stliq_id = vcl_dict.keys()
                if len(vcl_dict) >= 2:
                    stliq_id.remove(max(stliq_id))
                    stliq_id = max(stliq_id)
                else:
                    stliq_id = liq_id

                # if i am liq, send intro
                print 'liq'
                print liq_id
                print 'vcl_id'
                print vcl_id
                if liq_id == vcl_id:
                    print "Sending INTRO message"
                    mymsg = build_message_to_send(vcl_id, 'INTRO', vcl_dict)
                    ser.write(mymsg)
    
                # if this is the second hello from new vehicle AND i am second-to-liq, send intro, update dict, update liq
                elif hello_dict[liq_id+1]>= 2 and vcl_id == stliq_id:
                    vcl_dict[liq_id] = 0
                    #send intro
                    print "Sending INTRO message"
                    mymsg = build_message_to_send(vcl_id, 'INTRO', vcl_dict)
                    ser.write(mymsg)

            # UPDATE XBEE
            if rec_msg.message_type == 'UPDATE':
                print "I received UPDATE from ", rec_msg.vcl_id
                #Updates dictionary with spot chosen by rec_msg.vcl_id
                if rec_msg.vcl_id in vcl_dict:
                    temp_spot = vcl_dict[rec_msg.vcl_id]
                else:
                    temp_spot = rec_msg.data[rec_msg.vcl_id]
                vcl_dict[rec_msg.vcl_id] = rec_msg.data[rec_msg.vcl_id]
                new_spot = vcl_dict[rec_msg.vcl_id]
                liq_id = max(vcl_dict.keys())
                if isUI == True:
                    if temp_spot == 0 and new_spot == 0:
                        sendUIUpdate('in_queue', rec_msg.vcl_id, new_spot)
                    elif temp_spot == 0 and new_spot>0 and new_spot <25:
                        sendUIUpdate('parking', rec_msg.vcl_id, new_spot)
                    elif temp_spot>0 and temp_spot<25 and new_spot>0 and new_spot<25:
                        sendUIUpdate('parked', rec_msg.vcl_id, new_spot)
                    else:
                        sendUIUpdate('returned', rec_msg.vcl_id, new_spot)
                    print 'sending selected spot to UI'
    
            # PARKED XBEE
            if rec_msg.message_type == 'PARKED':
                print "I received PARKED from ", rec_msg.vcl_id
                vcl_dict[rec_msg.vcl_id] = rec_msg.data[rec_msg.vcl_id]
                if isUI == True:
                    sendUIUpdate('parked', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])
                    print 'sending PARKED status to UI'
            
            # GOODBYE XBEE
            if rec_msg.message_type == 'GOODBYE':
                print "I received GOODBYE from ", rec_msg.vcl_id
                
                print "original dict"
                print vcl_dict

                if rec_msg.vcl_id in vcl_dict:
                    vcl_dict.pop(rec_msg.vcl_id)

                print "updated dict"
                print vcl_dict
                
                if len(vcl_dict.keys()) > 0:
                    liq_id = max(vcl_dict.keys())

                if isUI == True:
                    sendUIUpdate('returned', rec_msg.vcl_id, 0)
                    print 'sending GOODBYE status to UI'


# main loop
t1 = Thread(target = callback)
t2 = Thread(target = listen)

while not rospy.is_shutdown():
    connect()
    global liq_id, vcl_id, hellocount
    liq_id = 5
    vcl_id = 0
    hellocount = 0
    
    time.sleep(5)
    t1.start()

    while vcl_id == 0 and hellocount<= 2:
        if hellocount == 2:
            vcl_id = 5
            vcl_dict = {5:0}  
            hellocount = 3
            print "I'm 5 now"
            # Publish existence to UI
            if isUI:
                sendUIUpdate('in_queue', vcl_id, vcl_spot)
                print "I'm the UI"
        else:
            hellocount = hellocount+1
            mymsg = build_message_to_send(vcl_id, 'HELLO', {0:0})
            ser.write(mymsg)
            print "I sent HELLO"
            time.sleep(3.5)
                #wait
    
    t2.start()
    signal.pause()
