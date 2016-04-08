#!/usr/bin/python

import os
import signal
from bluetooth import *
import rospy
from std_msgs.msg import String
from threading import Thread
import time
import socket

global runThreads
runThreads = False
global theadStarted
threadStarted = False

rospy.init_node("bluetooth_comm")
pub1 = rospy.Publisher("blue_pull", String, queue_size=10)
#time.sleep(15)

global server_sock
server_sock=BluetoothSocket( RFCOMM )
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind(("",1))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

advertise_service( server_sock, "SampleServer",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )

# sends data to phone over bluetooth
def callback(data):
    global client_sock
    if runThreads:
	if len(str(data)) != 0:
	    client_sock.send(data.data)
	    #client_sock.send(" aayyee ")
	    print("got something from node: %s" % data.data)

# listen to node and send to sock
def listen():
    rospy.Subscriber("blue_push", String, callback)
    rospy.spin()
    
# receive from sock and talk to node
def talk():
    global runThreads, client_sock
    while runThreads:
	try:
            data = client_sock.recv(1024)
	    if len(data) != 0:
                print data
	        pub1.publish(data)
	        client_sock.send(data)
	        time.sleep(.5)
        except IOError:
	    runThreads = False
	    print("Disconnected on talk")
	    print("Waiting for connection on RFCOMM channel %d" % port) 
	    #global client_sock
 	    client_sock, client_info = server_sock.accept()
	    print("Accepted connection from ", client_info)
	    runThreads = True
	    pass

def signal_handler(signal, frame):
    print("Bluetooth closing gracefully")
    global server_sock, client_sock
    try:
        server_sock.close()
    except NameError:
        print "server_sock not defined"
    try:
        client_sock.close()
    except NameError:
        print "client_sock not defined"

    global runThreads
    runTheads = False
    os.kill(os.getpid(), 9)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTSTP, signal_handler)

t1 = Thread(target=listen)
t2 = Thread(target=talk)
global client_sock
global client_info

#def main():

while not rospy.is_shutdown():
    while not runThreads:
	print("Waiting for connection on RFCOMM channel %d" % port) 
	#global client_sock
 	client_sock, client_info = server_sock.accept()
	print("Accepted connection from ", client_info)
	runThreads = True
	print("Run Threads True")
	try:
	    if threadStarted != True:
		print("starting threads")
		threadStarted = True
		t1.start()
		t2.start()
		signal.pause()
		print("yeah it gets here")
	except IOError: 
	    print("Disconnected from ", client_info)
	    runThreads = False
	    print("Run Threads False")

client_sock.close()
server_sock.close()
print("all done")
