#!/usr/bin/python


from bluetooth import *
import rospy
from std_msgs.msg import String

rospy.init_node("bluetooth_comm")
pub1 = rospy.Publisher("blue_pull", String, queue_size=10)

server_sock=BluetoothSocket( RFCOMM )
server_sock.bind(("",PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

advertise_service( server_sock, "SampleServer",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )
       
while not rospy.is_shutdown():
    print("Waiting for connection on RFCOMM channel %d" % port) 
      
    client_sock, client_info = server_sock.accept()
    print("Accepted connection from ", client_info)
    try:
        while True:
            data = client_sock.recv(1024)
            if len(data) == 0: break
            pub1.publish(data)
	    print data
	    client_sock.send("heeeyyyy")
	    print "heeyyyy"
    except IOError: 
        print("Disconnected from ", client_info)
        pass
pass

client_sock.close()
server_sock.close()
print("all done")