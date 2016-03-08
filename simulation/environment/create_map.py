import cv2
import numpy as np

im = np.zeros((360, 300, 1), np.uint8)
im[:, :] = 255
im[:, 0] = 0
im[:, 299] = 0
im[0, :] = 0
im[359, :] = 0

for i in range(20, 281, 20):
    im[0:30, i] = 0
    im[60:120, i] = 0
    im[150:210, i] = 0
    im[240:300, i] = 0
    im[330:360, i] = 0

for i in range(90, 271, 90):
    im[i, 20:280] = 0

cv2.imwrite("map.jpg", im)

print "Format = [x1, y1, x2, y2]"

print "Row 1"

ctr = 0
for i in range(0, 300, 20):
    ctr += 1
    coords = [i+1, 1, i + 19, 29]
    print "Spot ", ctr, ": ", coords

print "Row 2"

ctr = 0
for i in range(20, 280, 20):
    ctr += 1
    coords = [i+1, 60, i + 19, 89]
    print "Spot ", ctr, ": ", coords

print "Row 3"

ctr = 0
for i in range(20, 280, 20):
    ctr += 1
    coords = [i+1, 91, i + 19, 119]
    print "Spot ", ctr, ": ", coords

print "Row 4"

ctr = 0
for i in range(20, 280, 20):
    ctr += 1
    coords = [i+1, 150, i + 19, 179]
    print "Spot ", ctr, ": ", coords

print "Row 5"

ctr = 0
for i in range(20, 280, 20):
    ctr += 1
    coords = [i+1, 181, i + 19, 209]
    print "Spot ", ctr, ": ", coords

print "Row 6"

ctr = 0
for i in range(20, 280, 20):
    ctr += 1
    coords = [i+1, 240, i + 19, 269]
    print "Spot ", ctr, ": ", coords

print "Row 7"

ctr = 0
for i in range(20, 280, 20):
    ctr += 1
    coords = [i+1, 300, i + 19, 329]
    print "Spot ", ctr, ": ", coords

print "Row 8"

ctr = 0
for i in range(0, 300, 20):
    ctr += 1
    coords = [i+1, 330, i + 19, 359]
    print "Spot ", ctr, ": ", coords

