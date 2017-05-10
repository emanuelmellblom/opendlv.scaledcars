#!/bin/bash

# SYSTEM -------------------------------------------------------------------
#Turn off Auto focus
uvcdynctrl -d video0 -s 'Focus, Auto' 0 
# --------------------------------------------------------------------------

#od supercomponent
xterm -hold -e "docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsupercomponent --cid=111 --verbose=1" &
sleep 3

#Real camera with own built proxy
xterm -hold -e "xhost + && docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY --group-add video -v $HOME/DIT-168:/opt/configuration --device=/dev/video0:/dev/video0 seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/cameraproxy --cid=111 --freq=20" &
sleep 3

#Lanefollower (RealCar)
xterm -hold -e "xhost + && docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY --device=/dev/ttyACM0:/dev/ttyACM0 -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/scaledcars-lanefollower --cid=111 --freq=10" &
sleep 3
