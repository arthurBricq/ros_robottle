ros2 run robottle teleop &
P1=$!
ros2 run robottle controller1 &
P2=$!
#wait $P1 $P2
echo $P1 $P2
