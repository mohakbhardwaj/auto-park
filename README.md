# auto-park
Official repository for MRSD'16 project - Auto Park for Social Robots

roslaunch autopark system.launch suppress:=1 nav:=0 emergency:=0 usb:=0

These arguments can be passed to the launch file to 1) Get/suppress the print outputs on screen, 2) Turn navigation on/off, 3) Turn emergency on/off 4) Specify USB port of Arduino.
The arguments are optional and will default to the values given in the above command if not provided though command-line. 1 is True and 0 is False
