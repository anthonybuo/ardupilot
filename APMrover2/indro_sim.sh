#!/usr/bin/env bash
source indro_sim.sh
echo "Entering Indro Rover Simulator..."
cd ~/ardupilot/Tools/autotest
python sim_vehicle.py -v APMrover2 -j4 --map --console
