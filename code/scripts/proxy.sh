#!/bin/bash

#Simulator Camera 

#od supercomponent
xterm -hold -e "docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsupercomponent --cid=111 --verbose=1" &
sleep 3

#odsimvehicle
#xterm -hold -e "docker run -ti --rm --net=host seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsimvehicle --cid=111 --freq=10" &
#sleep 3

#odsimirus
#xterm -hold -e "xhost + && docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsimirus --cid=111 --freq=10" &
#sleep 3

#od cockpit
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odcockpit --cid=111" &
#sleep 3

#odsimcamera
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsimcamera --freq=10 --cid=111" &
#sleep 3

#Start camera proxy with normal camera
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv --group-add video --device=/dev/video1:/dev/video1 seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/opendlv.core/bin/opendlv-core-system-proxy-camera --cid=111 --freq=20" &
#sleep 3

#Start proxy test with odsimcamera
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv --group-add video --device=/home/ost/Desktop/opendv/docker/builds/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-master/opt/opendlv.scaledcars/bin/proxy seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/opendlv.core/bin/opendlv-core-system-proxy-camera --cid=111 --freq=20" &
#sleep 3

#TEST ODSIMCAMER PROXY
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv --group-add video --device=$HOME/DIT-168:/opt/configuration seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/opendlv.core/bin/opendlv-core-system-proxy-camera --cid=111 --freq=20" &
#sleep 3

#lane detector
#Template repository data triggerd example with openCV
xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY --device=/dev/ttyACM0:/dev/ttyACM0 -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/proxy --cid=111 --freq=40" &
sleep 3

#xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/scaledcars-perception-example --cid=111 --freq=10" &
#sleep 3

#Scenario box parker
#Change to box-parker in the configuration file
#xterm -hold -e "docker run -ti --rm --net=host seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/miniature/boxparker --cid=111 --freq=10" &
#sleep 3

#Start camera recording
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv -v ~/recordings:/opt/recordings -w /opt/recordings seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odrecorderh264 --cid=111" &
#sleep 3



#test
#docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/opendlv.scaledcars/bin/scaledcars-control-lanefollwer --cid=111 --freq=10
