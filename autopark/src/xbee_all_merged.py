#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String
import subprocess
import time
import os
import signal
from threading import Thread
from protocolwrapper import (
    ProtocolWrapper, ProtocolStatus)
from myformat import (
    message_format, message_crc, Container)
import numpy as np
import hashlib
import sys
from autopark.msg import list_ui

rospy.init_node("Xbee")
pub1 = rospy.Publisher("ui_update", list_ui, queue_size=10)
pub2 = rospy.Publisher("bs", String, queue_size=10) #binary spots to planner
pub3 = rospy.Publisher("destination", String, queue_size=10) # exit coordinates to navigation


def signal_handler(signal, frame):
    print("I sent GOODBYE")
    mymsg = build_message_to_send(vcl_id,'GOODBYE',{0:0})
    ser.write(mymsg)
    if isUI:
        msg=list_ui()
        msg.status='returned'
        msg.vcl_id=vcl_id
        msg.spot_id=25
        pub1.publish(msg) 
    time.sleep(1)
    os.kill(os.getpid(), 9)
    

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTSTP, signal_handler)

# global variables

global hellocount
PROTOCOL_HEADER = '$'
PROTOCOL_FOOTER = '$'
PROTOCOL_DLE = '\x90'
global vcl_id
vcl_dict={}
hello_dict={}
global liq_id
isUI = int(sys.argv[1])
if isUI==1:
    isUI=True
else:
    isUI=False
vcl_spot=0 #receive from planner


#port=str(sys.argv[2])
#ser = serial.Serial(port, 9600, timeout=0.5)
#ser.flushInput()


def connect():
    global ser, flag
    x = subprocess.check_output("ls /dev/serial/by-id", shell=True).split()
    if x[0].find("Digi") != -1:
        y = subprocess.check_output("dmesg | grep tty", shell=True).split()
        indices = [i for i, x in enumerate(y) if x == "FTDI"]
        address = "/dev/" + y[indices[-1]+8]
        ser = serial.Serial(address, 9600, timeout=1)
        ser.flushInput()
        ser.flushOutput()
        flag = 0

    else:
        raise ValueError

    return


def build_message_to_send(
        vcl_id, message_type,
         vcl_dict):
   
    data = dictToList(vcl_dict)
    datalen = len(data)
  
    raw = message_format.build(Container(
        vcl_id=vcl_id,      
        message_type=message_type,
        datalen=datalen,
        data=data,
        crc=0))
       
    msg_without_crc = raw[:-8]    
    msg_crc = message_crc.build(Container(
        crc=int(''.join([i for i in hashlib.md5(msg_without_crc).hexdigest() if i.isdigit()])[0:10])))
   
    msg = msg_without_crc + msg_crc
    
    pw = ProtocolWrapper(
            header=PROTOCOL_HEADER,
            footer=PROTOCOL_FOOTER,
            dle=PROTOCOL_DLE)

    return pw.wrap(msg)



def recover_msg(msg):
        pw = ProtocolWrapper(
        header=PROTOCOL_HEADER,
        footer=PROTOCOL_FOOTER,
        dle=PROTOCOL_DLE)

        status = map(pw.input, msg)
        rec_crc=0
        calc_crc=1
        
        if status[-1] == ProtocolStatus.MSG_OK:
                rec_msg = pw.last_message
                rec_crc = message_crc.parse(rec_msg[-8:]).crc
                calc_crc = int(''.join([i for i in hashlib.md5(rec_msg[:-8]).hexdigest() if i.isdigit()])[0:10])

        if rec_crc != calc_crc:
            print 'Error: CRC mismatch'
     

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
    vcl_dict={data[i]: data[i+1] for i in range(0, len(data), 2)}
    return vcl_dict

def getmsg():
    time.sleep(0.5) 
    getmsg = ser.read(ser.inWaiting())
    rec_msg = recover_msg(getmsg)    
    rec_msg= message_format.parse(rec_msg)
    rec_msg.data = listToDict(rec_msg.data)
    return rec_msg

def listen():
    rospy.Subscriber("xbee_update", String, callback_master)
    rospy.Subscriber("opt_spot", String, callback_planner)
    rospy.Subscriber("vv_update", String, callback_UI)
    rospy.spin()

def callback_master(data):
    global vcl_dict,isUI,vcl_spot

    print "heard something from master:"
    print data.data
    

    if data.data=='park':
        print "heard park"

        bs=[0]*24
        print bs
        for val in vcl_dict.values():
            print val
            if val!=25 and val!=0:
                bs[val-1]=1

        print "sending to bs"
        print bs
        pub2.publish(str(bs))

    elif data.data=='return':
        print "heard return"

        print "sending exit coordinates to nav"

        pub3.publish('exit coordinates')

        vcl_dict[vcl_id]=25
        vcl_spot=25
        if isUI:
            print "sending return to UI"
            msg=list_ui()
            msg.status='return'
            msg.vcl_id=vcl_id
            msg.spot_id=vcl_spot     #changed from 25
            pub1.publish(msg)
        else:
            print "Sending UPDATE message b/c ROS   received"
            mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
            ser.write(mymsg)

    elif data.data=='parked':
        print "heard parked"

        if isUI:
            print "sending parked to UI"
            msg=list_ui()
            msg.status='parked'
            msg.vcl_id=vcl_id
            msg.spot_id=vcl_spot
            pub1.publish(msg)
        else:
            print "Sending PARKED message b/c ROS parked received"
            mymsg = build_message_to_send(vcl_id, 'PARKED', {vcl_id:vcl_spot})
            ser.write(mymsg)

    elif data.data=='returned':
        print "heard returned"

        if isUI:
            print "sending returned to UI"
            msg=list_ui()
            msg.status='returned'
            msg.vcl_id=vcl_id
            msg.spot_id=25
            pub1.publish(msg)
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
    temp[0]=int(temp[0])
    temp[1]=int(temp[1])

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


# when spot chosen from planner, update local dict and send update to other XBees and UI
def callback_planner(data):
    #assuming incoming data is string containing spot chosen
    global vcl_dict, vcl_id, vcl_spot, isUI

    print "heard something from planner:"
    print data.data

    temp = data.data
    temp = int(temp)

    print "this is the spot i know and love"
    print temp

    print "original dict"
    print vcl_dict

    vcl_dict[vcl_id] = temp
    vcl_spot = temp

    print "updated dict"
    print vcl_dict

    if isUI:
        msg=list_ui()
        msg.status='parking'
        msg.vcl_id=vcl_id
        msg.spot_id=vcl_spot
        pub1.publish(msg)
        print 'sending selected spot to UI'

    # send update to other XBees
    print "Sending UPDATE because planner spot was selected and received"
    mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
    ser.write(mymsg)


def callback():
    print "entering callback"
    global liq, vcl_dict, liq_id, isUI, vcl_spot, hello_dict, vcl_id, hellocount

    while True:
        while ser.inWaiting()>0:
            print "received message"
            rec_msg=getmsg()
        
            if rec_msg.message_type=='INTRO':
                print "I received INTRO from ", rec_msg.vcl_id
                # if somehow two vehicles got the same ID OR
                # if intro vcl has id greater than that in local dict
                # then bump up person ID, replace dict and send update
                if rec_msg.vcl_id == vcl_id or rec_msg.vcl_id > liq_id:
                    print 'rec_msg.data'
                    print rec_msg.data
                    for key, val in rec_msg.data.iteritems():
                        vcl_dict[key]=val
                    liq_id = max(vcl_dict.keys())
                    vcl_id = liq_id + 1
                    vcl_dict[vcl_id]=vcl_spot
                    liq_id = vcl_id
                    print "I'm the fuggin liq whatchu gon do"
                    # send update with personal spot
                    # vcl_id, msg_type, data
                    print "Sending update because erroneous INTRO received"
                    mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
                    ser.write(mymsg)
                if vcl_id == 0:
                    vcl_id = max(rec_msg.data.keys())+1
                    vcl_dict = rec_msg.data
                    vcl_dict[vcl_id]=vcl_spot
                    liq_id = vcl_id
                    print "I'm the fuggin liq whatchu gon do"
                    print "Sending update because I have arrived, mofo"
                    mymsg = build_message_to_send(vcl_id, 'UPDATE', {vcl_id:vcl_spot})
                    ser.write(mymsg)
                    #Tell the UI that I, and all my friends, exist.
                    if isUI:
                        for key, val in vcl_dict.iteritems():
                            msg=list_ui()
                            if val==0:
                                msg.status='in_queue'
                            elif val==25:
                                msg.status='returning'
                            else:
                                msg.status='parked'
                            msg.vcl_id=key
                            msg.spot_id=val
                            pub1.publish(msg)



            if rec_msg.message_type=='HELLO':
                print "I received HELLO from ", rec_msg.vcl_id

                # add liq_id+1 to hellos or increment val if necessary
                if liq_id+1 in hello_dict:
                    hello_dict[liq_id+1]=hello_dict[liq_id+1]+1
                else:
                    hello_dict[liq_id+1]=1
    
                # find second to liq_id.  if len(dict) < 2, stliq_id is the liq_id
                stliq_id = vcl_dict.keys()
                if len(vcl_dict) >=2:
                    stliq_id.remove(max(stliq_id))
                    stliq_id=max(stliq_id)
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
                elif hello_dict[liq_id+1]>=2 and vcl_id == stliq_id:
                    vcl_dict[liq_id] = 0
                    #send intro
                    print "Sending INTRO message"
                    mymsg = build_message_to_send(vcl_id, 'INTRO', vcl_dict)
                    ser.write(mymsg)

            if rec_msg.message_type=='UPDATE':
                print "I received UPDATE from ", rec_msg.vcl_id
                #Updates dictionary with spot chosen by rec_msg.vcl_id
                if rec_msg.vcl_id in vcl_dict:
                    temp_spot=vcl_dict[rec_msg.vcl_id]
                else:
                    temp_spot=rec_msg.data[rec_msg.vcl_id]
                vcl_dict[rec_msg.vcl_id]=rec_msg.data[rec_msg.vcl_id]
                new_spot=vcl_dict[rec_msg.vcl_id]
                liq_id = max(vcl_dict.keys())
                if isUI == True:
                    msg=list_ui()
                    if temp_spot==0 and new_spot == 0:
                        msg.status='in_queue'
                    elif temp_spot==0 and new_spot>0 and new_spot <25:
                        msg.status='parking'
                    elif temp_spot>0 and temp_spot<25 and new_spot>0 and new_spot<25:
                        msg.status='parked'
                    else:
                        msg.status='return'

                    msg.vcl_id=rec_msg.vcl_id
                    msg.spot_id=new_spot
                    pub1.publish(msg)
                    print 'sending selected spot to UI'
    
            if rec_msg.message_type=='PARKED':
                print "I received PARKED from ", rec_msg.vcl_id
                vcl_dict[rec_msg.vcl_id]=rec_msg.data[rec_msg.vcl_id]
                if isUI == True:
                    msg=list_ui()
                    msg.status='parked'
                    msg.vcl_id=rec_msg.vcl_id
                    msg.spot_id=vcl_dict[rec_msg.vcl_id]
                    pub1.publish(msg)
                    print 'sending PARKED status to UI'
                    
            if rec_msg.message_type=='GOODBYE':
                print "I received GOODBYE from ", rec_msg.vcl_id
                
                print "original dict"
                print vcl_dict

                if rec_msg.vcl_id in vcl_dict:
                    vcl_dict.pop(rec_msg.vcl_id)

                print "updated dict"
                print vcl_dict

                liq_id = max(vcl_dict.keys())

                if isUI == True:
                    msg=list_ui()
                    msg.status='returned'
                    msg.vcl_id=rec_msg.vcl_id
                    msg.spot_id=0
                    pub1.publish(msg)
                    print 'sending GOODBYE status to UI'

                
t1 = Thread(target=callback)
t2 = Thread(target=listen)

while not rospy.is_shutdown():
    connect()
    global liq_id, vcl_id, hellocount
    hellocount=0
    vcl_id=0
    liq_id=5
    
    time.sleep(10)
    t1.start()

    while vcl_id==0 and hellocount<=2:
        if hellocount==2:
            vcl_id=5
            vcl_dict={5:0}  
            hellocount=3
            print "I'm 5 now"
            # Publish existence to UI
            if isUI:
                msg=list_ui()
                msg.status='in_queue'
                msg.vcl_id=vcl_id
                msg.spot_id=vcl_spot
                pub1.publish(msg)
                print "I'm the muhfuggin UI"
        
        else:
            hellocount=hellocount+1
            mymsg = build_message_to_send(vcl_id, 'HELLO', {0:0})
            ser.write(mymsg)
            print "I sent HELLO"
                
            time.sleep(3.5)
                #wait
    
    t2.start()
    signal.pause()
