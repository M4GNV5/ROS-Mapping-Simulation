#!/bin/bash

source ../devel/setup.bash

python3 ohm_mecanum_sim/scripts/ohm_mecanum_sim_node.py &
p0=$!
python3 sim_pos_tracker/src/sim_pos_tracker_node.py &
p1=$!
python3 sim_cloud_creator/src/sim_cloud_creator.py &
p2=$!

sleep 1
python3 sim_mapper/src/sim_mapper_node.py &
p3=$!

read -p "Press enter to exit"

kill -9 $p0
kill -9 $p1
kill -9 $p2
kill -9 $p3
