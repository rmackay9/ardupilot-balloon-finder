#!/bin/bash
#
# start_balloon_finder.sh
#
# This script starts the balloon finder scripts

# wait 2 seconds before starting
sleep 2

date
PATH=$PATH:/bin:/sbin:/usr/bin:/usr/local/bin
PATH=$PATH:$HOME/GitHub/ardupilot-balloon-finder
PATH=$PATH:$HOME/GitHub/ardupilot-balloon-finder/scripts
export PATH
echo "PATH:" $PATH

PYTHONPATH=$PYTHONPATH:$HOME/GitHub/ardupilot-balloon-finder/scripts
export PYTHONPATH
echo "PYTHONPATH:" $PYTHONPATH

cd $HOME/GitHub/ardupilot-balloon-finder/scripts

python balloon_strategy.py

echo "start_balloon_finder.sh done"
