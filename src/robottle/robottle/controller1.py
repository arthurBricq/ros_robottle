import rclpy
from rclpy.node import Node
import numpy as np 
import time
import sys

from interfaces.msg import Map, Position, Status
from std_msgs.msg import String

from robottle_utils import map_utils, controller_utils
from robottle_utils.rrt_star import RRTStar


AREA_THRESHOLD = 60000
MIN_DIST_TO_GOAL = 1
CONTROLLER_TIME_CONSTANT = 20

# Array containing zones to visit
TARGETS_TO_VISIT = [2,3]


class Controller1(Node):
    """
    This node visualize the map received from the SLAM node.
    This node is quite hard to use, because it slows down a lot the process. 
    It should not be used in live with the Jetson 
    """
    def __init__(self):
        super().__init__("controller1")

        # Create subscription for the map 
        self.subscription1 = self.create_subscription(Position,'robot_pos',
            self.listener_callback_position,1000)
        self.subscription1  

        # Create subscription for the robot position
        self.subscription2 = self.create_subscription(Map,'world_map',
            self.listener_callback_map,1000)
        self.subscription2  

        # Create subscription for the UART reader (get signals from MC)
        self.subscription3 = self.create_subscription(Status,'arduino_status',
            self.listener_arduino_status,1000)
        self.subscription3  
        
        # Create a publication for uart commands
        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)

        # keep track of where is the robot within the class
        self.x = 0
        self.y = 0
        self.theta = 0 
        
        # variable for the controller
        self.initial_zones_found = False
        self.zones = []

        # set saving state (if True, then it will save some maps to a folder when they can be analysed)
        args = sys.argv
        if len(args) == 1: # it means no argument was passed to the ros node
            self.is_saving = False
        else:
            self.is_saving = True
            self.map_name = args[1]
            self.saving_index = 0

        # STATE MACHINE
        # send a request for continuous rotation after waiting 1 second for UART node to be ready
        self.state = "initial_rotation"
        time.sleep(3)
        self.uart_publisher.publish(String(data = "r"))
        time.sleep(10)

        self.state = "travel_mode"
        self.current_target_index = 0


    def listener_callback_map(self, map_message):
        if self.state == "travel_mode": 
            self.travel_mode(map_message)


    def travel_mode(self, map_message):
        """Travel mode of the controller. 
        This function is called by the map listener's callback.
        """
        map_data = bytearray(map_message.map_data)

        # Once in a while, start the controller logic
        if int(map_message.index) % CONTROLLER_TIME_CONSTANT == 0: 
            ### Map analysis 
            # a. filter the map 
            m = map_utils.get_map(map_data)
            robot_pos = map_utils.pos_to_gridpos(self.x, self.y)
            binary = map_utils.filter_map(m)
            # b. get rectangle around the map
            corners, area, contours = map_utils.get_bounding_rect(binary)
            print(area)
            # c. find zones 
            path = []
            if not self.initial_zones_found and area > AREA_THRESHOLD:
               # corners found are valid and we can find the 'initial zones' 
                self.zones = map_utils.get_initial_zones(corners, robot_pos)
                self.initial_zones_found = True
                return
            if self.initial_zones_found: 
                # zones are ordered the following way 
                # (recycling area, zone2, zone3, zone4)
                new_zones = map_utils.get_zones_from_previous(corners, self.zones)
                self.zones = new_zones

                ### Path Planing
                # those targets are ordered points
                targets = map_utils.get_targets_from_zones(np.array(self.zones), target_weight = 0.7)
                goal = targets[self.current_target_index]
                rrt = RRTStar(
                        start = robot_pos,
                        goal = goal,
                        binary_obstacle = binary,
                        rand_area = [0, 500],
                        expand_dis = 50,
                        path_resolution = 1,
                        goal_sample_rate = 5,
                        max_iter = 500,
                        )
                path = rrt.planning(animation = False)

                ### Path Tracker
                # 1. end condition
                dist = controller_utils.get_distance(robot_pos, goal)
                if dist < MIN_DIST_TO_GOAL:
                    # end of travel mode
                    self.state = "random_search"
                    return
                # 2. Else, a new state machine takes place ! 
                




            # d. make and save the nice figure
            if self.is_saving:
                save_name = "/home/arthur/dev/ros/data/maps/rects/"+self.map_name+str(self.saving_index)+".png"
                map_utils.make_nice_plot(binary, save_name, robot_pos, self.theta, contours, corners, self.zones, np.array(path).astype(int))
                np.save("/home/arthur/dev/ros/data/maps/" + self.map_name + str(self.saving_index) +  ".npy", m)
                self.saving_index += 1



    def listener_callback_position(self, pos):
        """This function just receives the position and will update it to self variables. 
        All control logics are in the 'map' calback"""
        # receive the position from the SLAM
        self.x = pos.x / 1000
        self.y = pos.y / 1000
        self.theta = pos.theta

        
    def listener_arduino_status(self, status_msg):
        status = status_msg.status
        print("Controller status received : ", status)
        # state machine logic
        if self.state == "initial_rotation":
            if status == 0:
                print("Controller says. 'hm, let me see what i can do...'")
            elif status == 1:
                print("Controller says: 'great job bob, now let's get you movin around ! '")
                self.state = "travel_mode"
            elif status == 2:
                print("Controller says: 'ok, i am waiting for your answer' ")




def main(args=None):
    rclpy.init(args=args)
    node = Controller1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

