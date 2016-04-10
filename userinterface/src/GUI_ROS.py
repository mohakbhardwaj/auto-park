#!/usr/bin/python

import cv2
from threading import Thread
import time
import copy
from autopark.msg import list_ui
from std_msgs.msg import String
import rospy
from Tkinter import *
import tkMessageBox
import os
import signal
import atexit

# initialize window
window = Tk()
window.wm_withdraw()


# initialize ros node, template image and other constants
t = time.time()
size = 66
impath = os.path.join(os.path.dirname(__file__), 'demo_template.jpg')
img = cv2.imread(impath)
a = img
spot_id = {1: (175, 260), 2: (241, 260), 3: (307, 260), 4: (373, 260),
           5: (439, 260), 6: (505, 260), 7: (175, 326), 8: (241, 326),
           9: (307, 326), 10: (373, 326), 11: (439, 326), 12: (505, 326),
           13: (175, 524), 14: (241, 524), 15: (307, 524), 16: (373, 524),
           17: (439, 524), 18: (505, 524), 19: (175, 590), 20: (241, 590),
           21: (307, 590), 22: (373, 590), 23: (439, 590), 24: (505, 590),
           25: (0,0)}
queue = 0
returning = 0
rospy.init_node("GUI")
pub1 = rospy.Publisher("ui_update",list_ui, queue_size = 10)
pub2 = rospy.Publisher("vv_update", String, queue_size = 10)
msg = list_ui()
current_state = {}
virtualcount = 0
max_virtual_vehicles = 99
time.sleep(3)


# remove all virtual vehicles from parking lot when GUI is closed from terminal
def signal_handler(signal, frame):
    global virtualcount, max_virtual_vehicles
    for key in current_state.keys():
        if key <= max_virtual_vehicles:
            sendToXBee(key,25,0)
            time.sleep(1)
            msg.status = "returned"
            msg.vcl_id = key
            virtualcount -= 1
            msg.spot_id = 25
            pub1.publish(msg)
            print ("publishing to self: status: " + str(msg.status) + ". id: " + str(msg.vcl_id) + ", spot: " + str(msg.spot_id))
    cv2.destroyAllWindows()
    os.kill(os.getpid(), 9)


# define ctrl+c and ctrl+z as signals to quit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTSTP, signal_handler)


# compute the ID of the spot selected by the user
def pixel_to_spot(x, y):
    pos_id, id_x, id_y = (-1, -1, -1)
    if 142 <= x <= 603 and 227 <= y <= 425:
        for i in range(208, 604, 66):
            if i > x:
                id_x = i-33
                break
        for j in range(293, 426, 66):
            if j > y:
                id_y = j-33
                break
        for ID, pos in spot_id.iteritems():
            if pos == (id_x, id_y):
                pos_id = ID
    elif 142 <= x <= 603 and 491 <= y <= 623:
        for i in range(208, 604, 66):
            if i > x:
                id_x = i-33
                break
        for j in range(557, 624, 66):
            if j > y:
                id_y = j-33
                break
        for ID, pos in spot_id.iteritems():
            if pos == (id_x, id_y):
                pos_id = ID
                break
    elif 0 <= x < 142 and 0 <= y <= 95:
        #print "exit"
        pass
    elif 542 <= x < 672 and 761 <= y <= 799:
        #print "entrance"
        pos_id = 0
    return pos_id


# generate error messages
def popup(data):
    window.geometry("1x1+"+str(window.winfo_screenwidth()/2)+"+"+str(window.winfo_screenheight()/2))
    tkMessageBox.showinfo(title="Message", message=data)


# draws a virtual vehicle based on mouse click
def draw_virtual_vehicle(event, x, y, flag, param):
    global current_state, virtualcount, max_virtual_vehicles
    pos_id = pixel_to_spot(x, y)
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # print ("pos_id: " + str(pos_id))
        # print "Left Click"
        if pos_id != -1:
            if virtualcount == max_virtual_vehicles:
                popup("Error: Max Virtual Vehicles reached")
            else:
                if len(current_state.keys()) > 0 and min(current_state.keys()) < 100:
                    for i in range(1, max([v for v in current_state.keys() if v <= max_virtual_vehicles])+2):
                        if i not in current_state.keys():
                            id_vv = i
                            break
                    print ("id_vv: " + str(id_vv))
                else:
                    id_vv = 1
                    print ("id_vv: " + str(id_vv))

                states={}
                for key in current_state.keys():
                    if (pos_id == current_state[key][0] or pos_id == current_state[key][1]) and pos_id != 0:
                        id_vv = key
                        print ("id_vv: " + str(id_vv))
                    states[current_state[key][0]]=current_state[key][1]

                if pos_id in states:# and (states[pose_id] == pos_id or states[pos_id]==25)) and pos_id != 0: # if vehicle parked or vehicle returning
                    popup("Error: Spot is already occupied")
                elif id_vv > max_virtual_vehicles:
                    popup("Error: Cannot change physical vehicle")
                else:
                    if id_vv in current_state.keys():
                        if current_state[id_vv][1] == pos_id:
                            msg.status="parked"
                            motion = 0
                        elif pos_id > 0:
                            msg.status="parking"
                            motion = 1
                        else:
                            # assuming pos_id == 0: #in_queue
                            msg.status = "in_queue"
                            motion = 0
                    else:
                        if pos_id > 0:
                            msg.status="parking"
                            motion = 1
                        else:
                            # assuming pos_id == 0: #in_queue
                            msg.status = "in_queue"
                            motion = 0

                    msg.spot_id=pos_id
                    msg.vcl_id=id_vv
                    print ("publishing to self: status: " + str(msg.status) + ". id: " + str(msg.vcl_id) + ", spot: " + str(msg.spot_id))
                    pub1.publish(msg)
                    sendToXBee(id_vv,pos_id,motion)

        else:
            popup("Error: Click on appropriate region")
    elif event == cv2.EVENT_RBUTTONDBLCLK:
        # print "Right Click"
        if pos_id != -1:
            #print ("pos_id: " + str(pos_id))
            if len(current_state.keys()) > 0:
                state = current_state.values()
                try:
                    id_vv = 100
                    for key, val in current_state.iteritems():
                        if val[0] == pos_id and key <= max_virtual_vehicles:
                            id_vv = key
                    print ("id_vv: " + str(id_vv))
                    if id_vv <= max_virtual_vehicles:
                        if current_state[id_vv][1] == pos_id: # vcl is parked, transition to returning
                            print "returning"
                            msg.status = "returning"
                            msg.vcl_id = id_vv
                            virtualcount -= 1
                            msg.spot_id = 25
                            pub1.publish(msg)
                            print ("publishing to self: status: " + str(msg.status) + ". id: " + str(msg.vcl_id) + ", spot: " + str(msg.spot_id))
                            sendToXBee(msg.vcl_id,25,1)
                        elif current_state[id_vv][1] == 25: # vcl is returning, transition to returned
                            print "returned"
                            msg.status = "returned"
                            msg.vcl_id = id_vv
                            pub1.publish(msg)
                            print ("publishing to self: status: " + str(msg.status) + ". id: " + str(msg.vcl_id) + ", spot: " + str(msg.spot_id))
                            sendToXBee(msg.vcl_id,25,0)
                            #time.sleep(5)
                            #msg.status = "gone"
                            #pub1.publish(msg)
                            #print ("publishing to self: status: " + str(msg.status) + ". id: " + str(msg.vcl_id) + ", spot: " + str(msg.spot_id))
                    else:
                        popup("Error: Can't remove physical vehicles")
                except ValueError:
                    popup("Error: Can't remove vehicle from the selected spot")
            else:
                popup("Error: No vehicles to remove")
        else:
            popup("Error: Click on appropriate region")


# sends message to XBee
def sendToXBee(vcl_id, vcl_spot, in_motion):
    print ("sent to XBee: id: " + str(vcl_spot)+", spot: "+str(vcl_spot)+", motion: " + str(in_motion))
    pub2.publish(""+str(vcl_id)+", "+str(vcl_spot)+","+str(in_motion))


# draws a vehicle (circle) with its ID on the map
def draw_vehicle(idn, position, color):
    global a, max_virtual_vehicles
    if idn > max_virtual_vehicles + 1:
        off_x = 20
        off_y = 10
        f_size = 0.5
    else:
        off_x = 15
        off_y = 15
        f_size = 1
    str_x, str_y = spot_id[position]
    cv2.circle(a, spot_id[position], 25, color, thickness=6, lineType=8, shift=0)
    cv2.putText(a, str(idn), (str_x - off_x, str_y + off_y), cv2.FONT_HERSHEY_SIMPLEX, f_size, (0, 0, 0), 2)


# draws a vehicle (circle) with its ID on the entrance queue
def draw_queue_vehicle(idn, color):
    global a, queue, max_virtual_vehicles
    queue += 1
    if idn > max_virtual_vehicles + 1:
        off_x = 562
        off_y = 786
        f_size = 0.2
    else:
        off_x = 566
        off_y = 788
        f_size = 0.4
    cv2.circle(a, (queue*30 + 574, 780), 15, color, thickness=3, lineType=8, shift=0)
    cv2.putText(a, str(idn), (queue*30 + off_x, off_y), cv2.FONT_HERSHEY_SIMPLEX, f_size, (255, 255, 255), 1)


# draws a vehicle (circle) with its ID at the exit queue
def draw_returning_vehicle(idn, color):
    global a, returning, max_virtual_vehicles
    returning += 1
    if idn > max_virtual_vehicles + 1:
        off_x = -10
        off_y = 80
        f_size = 0.2
    else:
        off_x = -5
        off_y = 85
        f_size = 0.4
    cv2.circle(a, (returning*30 + 0, 80), 15, color, thickness=3, lineType=8, shift=0)
    cv2.putText(a, str(idn), (returning*30 + off_x, off_y), cv2.FONT_HERSHEY_SIMPLEX, f_size, (255, 255, 255), 1)


# decides a color for the vehicle based on the ID
def draw_vehicle_color(idn):
    r = abs(idn*60 + 60) % 255
    g = abs(idn*120 + 120) % 255
    b = abs(idn*180 + 180) % 255
    color = (b, g, r)
    return color


# shows the spot blocked by a vehicle intending to park itself there
def draw_occupied_spot(idn, position):
    global a, max_virtual_vehicles
    if idn > max_virtual_vehicles + 1:
        off_x = 33
        f_size = 0.9
    else:
        off_x = 20
        f_size = 1.2
    str_x, str_y = spot_id[position]
    cv2.putText(a, str(idn), (str_x - off_x, str_y + 20), cv2.FONT_HERSHEY_SIMPLEX, f_size, (0, 0, 0), 2)


# interprets the data coming in
def interpret(data):
    global a, queue, current_state, returning
    print "received message.  current state:"
    print current_state
    x_id = data.vcl_id
    x_pos = data.spot_id
    x_status = data.status
    if x_id in current_state:
        temp = [x_id,current_state[x_id][0]]
        del current_state[x_id]
    if x_status == "in_queue":
        print "inqueue"
        current_state[x_id] = [0,0]
    elif x_status == "parking":
        print "parking"
        current_state[x_id] = [0,x_pos]
    elif x_status == "parked":
        print "parked"
        current_state[x_id] = [x_pos,x_pos]
    elif x_status == "returning":
        print "returning"
        current_state[x_id] = [temp[1],25]
    elif x_status == "returned":
        print "returned"
        current_state[x_id] = [25,25]
    elif x_status == "gone":
        print "gone"
    print "updated current state:"
    print current_state
    a = copy.copy(img)
    queue = 0
    returning = 0
    gone = []
    for key in current_state.keys():
        val = current_state[key]
        color = draw_vehicle_color(key)
        if val[1] == 0:
            print ("vcl " + str(key) + " in queue")
            print "draw queue"
            draw_queue_vehicle(key, color)
        elif val[0] == 0 and val[1] != 25:
            print ("vcl " + str(key) + " parking")
            print "draw queue"
            draw_queue_vehicle(key, color)
            print "draw occupied"
            draw_occupied_spot(key, val[1])
        elif val[0] != 25 and val[1] != 25:
            print ("vcl " + str(key) + " parked")
            print "draw vehicle"
            draw_vehicle(key, val[0], color)
        elif val[0] != 25 and val[1] == 25:
            print ("vcl " + str(key) + " returning")
            print "draw exit"
            draw_returning_vehicle(key, color)
            print "draw occupied"
            draw_occupied_spot(key, val[0])
        elif val[0] == 25 and val[1] == 25:
            print ("vcl " + str(key) + " returned")
            print "draw exit"
            draw_returning_vehicle(key, color)
            gone.append(key)
    time.sleep(1)
    for vcl in gone:
        msg.status = "gone"
        msg.vcl_id = vcl
        msg.spot_id = 25
        pub1.publish(msg)



# access data from the network
def update():
    rospy.Subscriber("ui_update", list_ui, interpret)
    rospy.spin()


# start the thread responsible for updating the image
t1 = Thread(target=update)
t1.start()


# refreshes the image at regular intervals
cv2.namedWindow('AutoPark', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('AutoPark', draw_virtual_vehicle)
while True:
    cv2.imshow('AutoPark', a)
    cv2.waitKey(20) & 0xFF
