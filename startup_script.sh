# source ROS2
. install/setup.sh

# launch all the nodes
ros2 run robottle teleop &
P1=$!
ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0
P2=$!
ros2 launch robottle bottle_picking.launch.py
P3=$!
#wait $P1 $P2
echo $P1 $P2 $P3
