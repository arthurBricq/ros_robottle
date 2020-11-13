# Robottle Controller 1

First Robot Autonomous Controller is a State Machine performing Random Walk to Collect bottles. This files reports a documentation of this controller and lays out what needs to be done.

![controller](imgs/controller1.png)

## State Machine Description

### Travel Mode

In this state, the robot must travel to another zone - where we know it's going to make more points. Several situations can occurs:
1. The robot will go pick bottles in another new (*not explored*) zone: **exploration mode**. For now, we assume in this mode that the robot is in the recycling area and that it is empty. However, we don't assume any orientation of the robot. 
2. The robot will come back at the **recycling area** to drop its bottles: **return mode** 


What information does the algorithm needs to **make the robot travel to the desired zone** ?
- (i1) = **the desired zone**: it is easy to get those, we simply keep an array `desired_zones` and `visited_zones`, then make a substraction of those 2 sets to extract where we want to go.
- (i2) = **robot state & environment**: it is the output of the SLAM (*needs an evaluation*)
- (i3) = **position of the desired end position in the environment**: this is the hardest part. We must have a desired position in term of the occupancy grid, but the occupancy is a variable that we do not control and that evolves as the robot moves. 

**How can we find (i3)**

First thing to be said is that (i3) is not a constant and **it will be re-evaluated** as the robot moves. **There is a strong required coupling between path planning and path following**, and this is the case because we have a **dynamic map construction**. 

So what data can we use to deduce (i3) if we assume to have at disposition (i1) + (i2)...
- Prior knowledge of the **real map**
    - the real map is a square
    - the real map contains a ramp in a given position
    - the real map has 4 color beacons
- Vision *service* (in the ROS sense) to take a picture, and returns the orientation towards one beacon if one is detected (this service is going to be called at a certain frequency)

**Path planning and path following**

As explained before, those 2 functions are higlhy coupled. Hence, we choose as approach a **potential-based** path planner, with **attraction force** coming from the desired position and some **repulsion forces** coming from obstacles. Once the resulting force is computed, the robot motor controller will use the **overall force direction** to navigate in the world.


### Random walk Mode


### Bottle Picking Mode
