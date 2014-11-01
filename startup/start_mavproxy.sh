#!/bin/bash
#
# start_mavproxy.sh
#
# This script starts up mavproxy as odroid and should be called from /etc/rc.local

# give 10 seconds before starting
sleep 10

date
export HOME=/home/odroid
PATH=$PATH:/bin:/sbin:/usr/bin:/usr/local/bin
PATH=$PATH:$HOME/GitHub/ardupilot-balloon-finder
PATH=$PATH:$HOME/GitHub/ardupilot-balloon-finder/scripts
PATH=$PATH:$HOME/GitHub/ardupilot-balloon-finder/smart_camera
export PATH
echo "PATH:" $PATH

PYTHONPATH=$PYTHONPATH:$HOME/GitHub/ardupilot-balloon-finder/scripts
export PYTHONPATH
echo "PYTHONPATH:" $PYTHONPATH

cd $HOME
echo $HOME

ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id
ls -l /dev/video*

screen -d -m -s /bin/bash mavproxy.py --master=/dev/ttyUSB0 --baudrate 921600 --aircraft MyCopter

echo "start_mavproxy.sh done"
