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
global vcl_motion
vcl_dict = {}
hello_dict = {}
vcl_spot = 0 #receive from planner
vcl_motion = 0 # starts in_queue, not moving
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


# builds and sends message to XBees
def send_XBee_msg(vcl_id, message_type, vcl_dict):
    print ("sending " + message_type + " message from vehicle " + str(vcl_id) + " to XBees")
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


# send GOODBYE to other XBees when ctrl+c or ctrl+z heard, then shut down
def signal_handler(signal, frame):
    #print("I sent GOODBYE")
    print("XBee closing gracefully")
    msg = send_XBee_msg(vcl_id,'GOODBYE',{0:[0,0]})
    ser.write(msg)
    if isUI:
        sendUIUpdate('returned', vcl_id, [25,0])
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
            return None
        else:
            return rec_msg


# reads message from XBees, parses, and returns it
def getmsg():
    time.sleep(0.5) 
    getmsg = ser.read(ser.inWaiting())
    rec_msg = recover_msg(getmsg)
    if rec_msg is not None:
        rec_msg = message_format.parse(rec_msg)
        rec_msg.data = listToDict(rec_msg.data)
        return rec_msg
    else:
        return None


# Returns int list of key, vals in vcl_dict
def dictToList(vcl_dict):
    data = []
    for key, val in vcl_dict.iteritems():
        data.append(key)
        data.append(val[0])
        data.append(val[1])
    return data


# Returns dictionary of int values in data
def listToDict(data):
    vcl_dict = {}
    vcl_dict = {data[i]: [data[i+1],data[i+2]] for i in range(0, len(data), 3)}
    return vcl_dict


# sends update to UI
def sendUIUpdate(stat, vcl, spot):
    print ("sending " + stat + " message from vehicle " + str(vcl) + " to UI")
    msg = list_ui()
    msg.status = stat
    msg.vcl_id = vcl
    msg.spot_id = spot[0]
    pub1.publish(msg)


# creates listeners to master and UI
def listen():
    rospy.Subscriber("xbee_update", String, callback_master)
    rospy.Subscriber("vv_update", String, callback_UI)
    rospy.spin()


# messages from app or ...locomotion?
def callback_master(data):
    global vcl_dict, vcl_spot, vcl_motion, isUI
    print "heard something from master:"
    print data.data

    print "original dict"
    print vcl_dict

    # park master
    if data.data == 'park':
        print "heard park"

        bs = [0]*24
        print bs
        for val in vcl_dict.values():
            print val
            if val[0]!= 25 and val[0]!= 0:
                bs[val[0]-1] = 1

        print "the bs:"
        print bs

        #start planner
        # val_spot in format (spot cost, spot id - 1)
        val_spot = [(5, 0),  (6, 1),  (7, 2), (8, 3), (9, 4) , (10, 5), (9, 6),  (10, 7), (11, 8), (12, 9), (13, 10), (14, 11), (10, 12), (11, 13), (12, 14), (13, 15), (14, 16) ,(15, 17), (14, 18), (15, 19), (16, 20), (17, 21), (18, 22), (19, 23)]

        heapify(val_spot)
        found = False
        fail = False
        while found == False and fail == False:
                curr_spot = heappop(val_spot)

                spot_val, input_idx = curr_spot

                if bs[input_idx] == 0:
                        optim_spot = curr_spot
                        found = True
                        print "found spot"
                if len(val_spot) == 0:
                        print "Failed"
                        fail = True
        time.sleep(1)
        print 'opt spot ID'
        print optim_spot[1]+1
        print pub2.publish(repr(optim_spot[1]+1))
        #end planner

        #start callback

        vcl_spot = optim_spot[1]+1
        vcl_motion = 1
        vcl_dict[vcl_id] = [vcl_spot,vcl_motion]

        if isUI:
            print 'sending selected spot to UI'
            sendUIUpdate('parking', vcl_id, vcl_dict[vcl_id])

        print "Sending UPDATE because optimal spot was selected"
        msg = send_XBee_msg(vcl_id, 'UPDATE', {vcl_id:vcl_dict[vcl_id]})
        ser.write(msg)
        # end callback

    # return master
    elif data.data == 'return':
        print "heard return"
        print "sending exit ID (25) to nav"
        pub2.publish('25')

        vcl_motion = 1
        vcl_spot = 25
        vcl_dict[vcl_id] = [vcl_spot, vcl_motion]

        if isUI:
            sendUIUpdate('returning', vcl_id, vcl_dict[vcl_id])
        print "Sending UPDATE message b/c return command received"
        msg = send_XBee_msg(vcl_id, 'UPDATE', {vcl_id:vcl_dict[vcl_id]})
        ser.write(msg)

    # parked master
    elif data.data == 'parked':
        print "heard parked"

        vcl_motion = 0
        vcl_dict[vcl_id][1] = vcl_motion
        
        if isUI:
            sendUIUpdate('parked', vcl_id, vcl_dict[vcl_id])
        print "Sending PARKED message b/c ROS parked status received"
        msg = send_XBee_msg(vcl_id, 'PARKED', {vcl_id:vcl_dict[vcl_id]})
        ser.write(msg)

    # returned master
    elif data.data == 'returned':
        print "heard returned"

        if isUI:
            sendUIUpdate('returned', vcl_id, [25,0])
        print "Sending GOODBYE message b/c ROS returned received"
        msg = send_XBee_msg(vcl_id, 'GOODBYE', {0:[0,0]})
        ser.write(msg)
        print "Shutting down"
        os.kill(os.getpid(), 9)
    
    else:
        print "ERROR: received unrecognized data from Master"

    print "updated dict"
    print vcl_dict


# when msg received from UI, update local dict and send update to other XBees
# Virtual vehicles
def callback_UI(data):
    global vcl_dict

    print "heard something from UI:"
    print data.data

    temp = data.data
    temp = temp.split(',')
    ui_vcl_id = int(temp[0])
    ui_spot_id = int(temp[1])
    ui_motion = int(temp[2])

    print "key and val:"
    print ui_vcl_id
    print ui_spot_id
    print ui_motion
    print "original dict"
    print vcl_dict

    vcl_dict[ui_vcl_id] = [ui_spot_id, ui_motion]

    print "updated dict"
    print vcl_dict

    if ui_spot_id == 25 and ui_motion == 0:
        print "Sending GOODBYE b/c UI VV returned"
        msg = send_XBee_msg(ui_vcl_id, 'GOODBYE', {0:[0,0]})
        ser.write(msg)
        if ui_vcl_id in vcl_dict:
            del vcl_dict[ui_vcl_id]
            print "updated dict"
            print vcl_dict
    else:
        print "Sending UPDATE b/c UI VV message received"
        msg = send_XBee_msg(ui_vcl_id, 'UPDATE', {ui_vcl_id:vcl_dict[ui_vcl_id]})
        ser.write(msg)


# messages from other XBees
def callback():
    print "entering callback"
    global liq, vcl_dict, liq_id, isUI, vcl_spot, hello_dict, vcl_id, vcl_motion, hellocount

    while True:
        while ser.inWaiting()>0:
            print "received message"
            rec_msg = getmsg()
            if rec_msg is not None:
                print "original dict"
                print vcl_dict

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
                        vcl_dict[vcl_id] = [vcl_spot, vcl_motion]
                        liq_id = vcl_id
                        print "I'm the liq"
                        print "Sending update because erroneous INTRO received"
                        msg = send_XBee_msg(vcl_id, 'UPDATE', {vcl_id:vcl_dict[vcl_id]})
                        ser.write(msg)
                    if vcl_id == 0:
                        vcl_id = max(rec_msg.data.keys())+1
                        vcl_dict = rec_msg.data
                        vcl_dict[vcl_id] = [vcl_spot, vcl_motion]
                        liq_id = vcl_id
                        print "I'm the liq"
                        print "Sending update because I have arrived"
                        msg = send_XBee_msg(vcl_id, 'UPDATE', {vcl_id:vcl_dict[vcl_id]})
                        ser.write(msg)
                        #Tell the UI that I, and all my friends, exist.
                        if isUI:
                            for key, val in vcl_dict.iteritems():
                                msg = list_ui()
                                if val[0] == 0:
                                    sendUIUpdate('in_queue', key, val)
                                elif val[0] == 25 and val[1] == 1:
                                    sendUIUpdate('returning', key, val)
                                elif val[0] == 25 and val[1] == 0:
                                    sendUIUpdate('returned', key, val)
                                elif val[1] == 1: #moving and spot != 0 and spot != 25
                                    sendUIUpdate('parking', key, val)
                                elif val[1] == 0:
                                    sendUIUpdate('parked', key, val)
                                else:
                                    print ("ERROR: unknown status in dict: " + str(key) + " " + str(val))

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
                        msg = send_XBee_msg(vcl_id, 'INTRO', vcl_dict)
                        ser.write(msg)

                    # if this is the second hello from new vehicle AND i am second-to-liq, send intro, update dict, update liq
                    elif hello_dict[liq_id+1]>= 2 and vcl_id == stliq_id:
                        vcl_dict[liq_id] = [0,0]
                        msg = send_XBee_msg(vcl_id, 'INTRO', vcl_dict)
                        ser.write(msg)

                # UPDATE XBEE
                if rec_msg.message_type == 'UPDATE':
                    print "I received UPDATE from ", rec_msg.vcl_id

                    vcl_dict[rec_msg.vcl_id] = rec_msg.data[rec_msg.vcl_id]
                    liq_id = max(vcl_dict.keys())

                    if isUI:
                        if vcl_dict[rec_msg.vcl_id][0] == 0:
                            sendUIUpdate('in_queue', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])
                        elif vcl_dict[rec_msg.vcl_id][0] < 25 and vcl_dict[rec_msg.vcl_id][1] == 1:
                            sendUIUpdate('parking', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])
                        elif vcl_dict[rec_msg.vcl_id][0] < 25 and vcl_dict[rec_msg.vcl_id][1] == 0:
                            sendUIUpdate('parked', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])
                        elif vcl_dict[rec_msg.vcl_id][0] == 25 and vcl_dict[rec_msg.vcl_id][1] == 1:
                            sendUIUpdate('returning', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])
                        elif vcl_dict[rec_msg.vcl_id][0] == 25 and vcl_dict[rec_msg.vcl_id][1] == 0:
                            sendUIUpdate('returned', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])
                        else:
                            print ("ERROR: unknown status received: " + str(rec_msg.vcl_id) + " " + str(vcl_dict[rec_msg.vcl_id]))

                # PARKED XBEE
                if rec_msg.message_type == 'PARKED':
                    print "I received PARKED from ", rec_msg.vcl_id
                    vcl_dict[rec_msg.vcl_id] = rec_msg.data[rec_msg.vcl_id]
                    if isUI:
                        sendUIUpdate('parked', rec_msg.vcl_id, vcl_dict[rec_msg.vcl_id])

                # GOODBYE XBEE
                if rec_msg.message_type == 'GOODBYE':
                    print "I received GOODBYE from ", rec_msg.vcl_id

                    if rec_msg.vcl_id in vcl_dict:
                        vcl_dict.pop(rec_msg.vcl_id)


                    if len(vcl_dict.keys()) > 0:
                        liq_id = max(vcl_dict.keys())

                    if isUI:
                        sendUIUpdate('returned', rec_msg.vcl_id, [0, 0])

                print "updated dict"
                print vcl_dict
            else:
                print "ERROR RECEIVED BAD DATA"



# main loop
t1 = Thread(target = callback)
t2 = Thread(target = listen)

while not rospy.is_shutdown():
    connect()
    global liq_id, vcl_id, hellocount
    liq_id = 100
    vcl_id = 0
    hellocount = 0
    
    time.sleep(5)
    t1.start()

    while vcl_id == 0 and hellocount<= 2:
        if hellocount == 2:
            vcl_id = liq_id
            vcl_dict = {vcl_id:[0,0]}
            hellocount = 3
            print "I'm 100 now"
            # Publish existence to UI
            if isUI:
                sendUIUpdate('in_queue', vcl_id, vcl_dict[vcl_id])
                print "I'm the UI"
        else:
            hellocount = hellocount+1
            msg = send_XBee_msg(vcl_id, 'HELLO', {0:[0,0]})
            ser.write(msg)
            time.sleep(3.5)

    t2.start()
    signal.pause()