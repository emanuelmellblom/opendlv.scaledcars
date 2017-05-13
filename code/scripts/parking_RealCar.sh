#!/bin/bash

#od supercomponent
xterm -hold -e "docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsupercomponent --cid=111 --verbose=1" &
sleep 3

#OWN COMPONENTS ------------------------------------------------------------

#Sensor Proxy
xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY --device=/dev/ttyACM0:/dev/ttyACM0 -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/proxy --cid=111 --freq=40" &
sleep 3

#Parker
xterm -hold -e "xhost + && docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/Parking --cid=111 --freq=10" &
sleep 3
