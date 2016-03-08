from __future__ import division

import cv2
import numpy as np

im = np.zeros((1920, 1700, 1), np.uint8)
im[:, :] = 255
im[:, 0] = 0
im[:, 1699] = 0
im[0, :] = 0
im[1919, :] = 0

for i in xrange(200, 1550, 100):
    im[0:140, i] = 0
    im[340:620, i] = 0
    im[820:1100, i] = 0
    im[1300:1580, i] = 0
    im[1780:1920, i] = 0

for i in range(480, 1920, 480):
    im[i, 200:1500] = 0

cv2.imwrite("map.jpg", im)

with open("Spots.txt", "w") as text_file:

    for i in range(250, 1500, 100):
        string_spot = str(i/40) + " " + str(70/40)
        text_file.write("{0} Empty\n".format(string_spot))

    for x in range(410, 1800, 480):
        for y in range(x, x+150, 140):
            for i in range(250, 1500, 100):
                string_spot = str(i/40) + " " + str(y/40)
                text_file.write("{0} Empty\n".format(string_spot))

    for i in range(250, 1500, 100):
        string_spot = str(i/40) + " " + str(1850/40)
        text_file.write("{0} Empty\n".format(string_spot))
        
