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

# initialize window
window = Tk()
window.wm_withdraw()

# initialize ros node, template image and other constants
t = time.time()
size = 66
img = cv2.imread("demo_template.jpg")
a = img
spot_id = {1: (175, 260), 2: (241, 260), 3: (307, 260), 4: (373, 260),
           5: (439, 260), 6: (505, 260), 7: (175, 326), 8: (241, 326),
           9: (307, 326), 10: (373, 326), 11: (439, 326), 12: (505, 326),
           13: (175, 524), 14: (241, 524), 15: (307, 524), 16: (373, 524),
           17: (439, 524), 18: (505, 524), 19: (175, 590), 20: (241, 590),
           21: (307, 590), 22: (373, 590), 23: (439, 590), 24: (505, 590)}
queue = 0
rospy.init_node("GUI")
pub1 = rospy.Publisher("ui_update",list_ui, queue_size = 10)
pub2 = rospy.Publisher("vv_update", String, queue_size = 10)
msg = list_ui()
current_state = []
virtualcount = 0
time.sleep(10)


# compute the ID of the spot selected by the user
def pixel_to_spot(x, y):
    pos_id, id_x, id_y = (-1, -1, -1)
    if 603 >= x >= 142 and 425 >= y >= 227:
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
    elif 603 >= x >= 142 and 623 >= y >= 491:
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
    return pos_id


# generate error messages
def popup(data):
    window.geometry("1x1+"+str(window.winfo_screenwidth()/2)+"+"+str(window.winfo_screenheight()/2))
    tkMessageBox.showinfo(title="Message", message=data)


# draws a virtual vehicle based on mouse click
def draw_virtual_vehicle(event, x, y, flag, param):
    global current_state, virtualcount
    pos_id = pixel_to_spot(x, y)
    # print x, y, pos_id
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # print "Left Click"
        if pos_id != -1:
            if len(current_state) > 1:
                state = current_state[1:len(current_state):2]
                if pos_id in state:
                    popup("Error: Spot is already occupied")
                else:
                    ids = current_state[0:len(current_state):2]
                    if virtualcount == 4:
                        popup("Error: Max Virtual Vehicles reached")
                    else:
                        virtualcount += 1
                        msg.status = "parked"
                        id_vv = 1
                        while True:
                            if id_vv in ids:
                            	id_vv += 1
                            else:
                            	break
                        msg.vcl_id = id_vv
                        msg.spot_id = pos_id
                        pub1.publish(msg)
			pub2.publish(str(id_vv)+", "+str(pos_id))
            else:
            	virtualcount += 1
                msg.status = "parked"
                msg.vcl_id = 1
                msg.spot_id = pos_id
                pub1.publish(msg)
		pub2.publish(str(1)+", "+str(pos_id))
        else:
            popup("Error: Click on appropriate region")
    elif event == cv2.EVENT_RBUTTONDBLCLK:
        # print "Right Click"
        if pos_id != -1:
            if len(current_state) > 1:
                state = current_state[1:len(current_state):2]
                try:
                    ind = state.index(pos_id)
                    if current_state[2*ind] < 5:
                        msg.status = "returned"
                        msg.vcl_id = current_state[2*ind]
                    	virtualcount -= 1
	                msg.spot_id = 25
	                pub1.publish(msg)
			pub2.publish(str(msg.vcl_id)+", "+str(25))
			time.sleep(2)
			pub2.publish(str(msg.vcl_id)+", "+str(0))
		    else:
		    	popup("Error: Can't remove physical vehicles")
                except ValueError:
                    popup("Error: Can't remove vehicle from the selected spot")
            else:
                popup("Error: No vehicles to remove")
        else:
            popup("Error: Click on appropriate region")


# draws a vehicle (circle) with it's ID on the map
def draw_vehicle(idn, position, color):
    global a
    if idn > 9:
        off_x = 20
        off_y = 10
        f_size = 1
    else:
        off_x = 15
        off_y = 15
        f_size = 1.5
    str_x, str_y = spot_id[position]
    cv2.circle(a, spot_id[position], 25, color, thickness=6, lineType=8, shift=0)
    cv2.putText(a, str(idn), (str_x - off_x, str_y + off_y), cv2.FONT_HERSHEY_SIMPLEX, f_size, (0, 0, 0), 2)


# draws a vehicle (circle) with it's ID on the entrance queue
def draw_queue_vehicle(idn, color):
    global a, queue
    queue += 1
    if idn > 9:
        off_x = 562
        off_y = 786
        f_size = 0.6
    else:
        off_x = 566
        off_y = 788
        f_size = 0.8
    cv2.circle(a, (queue*30 + 574, 780), 15, color, thickness=3, lineType=8, shift=0)
    cv2.putText(a, str(idn), (queue*30 + off_x, off_y), cv2.FONT_HERSHEY_SIMPLEX, f_size, (255, 255, 255), 1)


# decides a color for the vehicle based on the ID
def draw_vehicle_color(idn):
    r = abs(idn*60 + 60) % 255
    g = abs(idn*120 + 120) % 255
    b = abs(idn*180 + 180) % 255
    color = (b, g, r)
    return color


# shows the spot blocked by a vehicle intending to park itself there
def draw_occupied_spot(idn, position):
    global a
    if idn > 9:
        off_x = 33
        f_size = 1.7
    else:
        off_x = 20
        f_size = 2
    str_x, str_y = spot_id[position]
    cv2.putText(a, str(idn), (str_x - off_x, str_y + 20), cv2.FONT_HERSHEY_SIMPLEX, f_size, (0, 0, 0), 2)


# interprets the data coming in
def interpret(data):
    global a, queue, current_state
    x_id = data.vcl_id
    x_pos = data.spot_id
    x_status = data.status
    if x_id in current_state[0:-1:2]:
        pop_i = current_state.index(x_id)
        current_state.pop(pop_i)
        temp_spot = current_state.pop(pop_i)
    if x_status == "in_queue" or x_status == "parked":
        current_state.append(x_id)
        current_state.append(x_pos)
    elif x_status == "parking":
        current_state.append(x_id)
        current_state.append([0, x_pos])
    elif x_status == "return":
        current_state.append(x_id)
        current_state.append([25, temp_spot])
    a = copy.copy(img)
    ids = current_state[0:len(current_state):2]
    state = current_state[1:len(current_state):2]
    queue = 0
    for i in range(0, len(ids)):
        color = draw_vehicle_color(ids[i])
        if type(state[i]).__name__ == 'int':
            if state[i] == 0:
                draw_queue_vehicle(ids[i], color)
            else:
                draw_vehicle(ids[i], state[i], color)
        elif type(state[i]).__name__ == 'list':
            if state[i][0] == 0:
                draw_queue_vehicle(ids[i], color)
                draw_occupied_spot(ids[i], state[i][1])
            if state[i][0] == 25:
                draw_occupied_spot(ids[i], state[i][1])


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
