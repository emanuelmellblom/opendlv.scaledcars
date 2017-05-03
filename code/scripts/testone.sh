#!/bin/bash

#od supercomponent
#xterm -hold -e "docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsupercomponent --cid=111 --verbose=1" &
#sleep 3

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

#Start camera proxy
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv --group-addvideo --device=/dev/video0:/dev/video0 seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/opendlv.core/bin/opendlv-core-system-proxy-camera --cid=111 --freq=20" &
#sleep 3

#Start proxy test with odsimcamera
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv --group-addvideo --device=/opt/od4/bin/odsimcamera seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/opendlv.core/bin/opendlv-core-system-proxy-camera --cid=111 --freq=20" &
#sleep 3

#Start camera recording
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host --user=odv -v ~/recordings:/opt/recordings -w /opt/recordings seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odrecorderh264 --cid=111" &
#sleep 3

#lane detector
#xterm -hold -e "docker run -ti --rm --net=host seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/miniature/lanedetector --cid=111 --freq=10" &
#sleep 3

#Scenario overtaker
#xterm -hold -e "docker run -ti --rm --net=host seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/miniature/overtaker --cid=111 --freq=10" &
#sleep 3

#Scenario box parker
#Change to box-parker in the configuration file
#xterm -hold -e "docker run -ti --rm --net=host seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/miniature/boxparker --cid=111 --freq=10" &
#sleep 3

#Scenarios editor
sudo xhost + && docker run -t -i --rm -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/$USER/DIT-168 -w /opt/ScenarioEditor seresearch/scenario_editor:v2 ./ScUI
sleep 3

#Template repository time triggerd example
#xterm -hold -e "docker run -ti --rm --net=host seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/opendlv.scaledcars/bin/scaledcars-control-example --cid=111 --freq=10" &
#sleep 3

#TEST DERIVER
#xterm -hold -e "docker run -ti --rm --net=host seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/driver --cid=111 --freq=10" &
#sleep 3

#TEST LANEFOLLOWER
#xterm -hold -e "docker run -ti --rm --net=host seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/lanefollower --cid=111 --freq=15" &
#sleep 3

#test driver container
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-onopendavinci-ubuntu-16.04-complete:latest /opt/opendlv.scaledcars/bin/driver --cid=111" &
#sleep 3

#Template repository data triggerd example with openCV
#xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev /opt/opendlv.scaledcars/bin/scaledcars-perception-example --cid=111" &
#sleep 3
