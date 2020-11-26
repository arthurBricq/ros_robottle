import rclpy
from rclpy.node import Node
import numpy as np 
import time

from interfaces.msg import Map, Position, Status
from std_msgs.msg import String

from robottle_utils import map_utils


AREA_THRESHOLD = 60000


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

        # set saving state (if True, then it will save some maps to a folder when they can be analysed)
        args = sys.argv
        if len(args) == 1: # it means no argument was passed to the ros node
            self.is_saving = False
        else:
            self.is_saving = True
            self.map_name = args[1]
            self.saving_index = 0

        # STATE MACHINE
        self.state = "initial_rotation"
        # send a request for continuous rotation after waiting 1 second for UART node to be ready
        time.sleep(3)
        self.uart_publisher.publish(String(data = "r"))


    def listener_callback_map(self, map_message):
        map_data = bytearray(map_message.map_data)

        # Once in a while, start an analysis
        if int(map_message.index) % 20 == 0: :
            ### Map analysis happening here

            # a. filter the map 
            m = map_utils.get_map(map_data)
            robot_pos = map_utils.pos_to_gridpos(self.x, self.y)
            binary = map_utils.filter_map(m)
            # b. get rectangle around the map
            corners, area, contours = map_utils.get_bounding_rect(binary)
            print(area)
            # c. find zones 
            if not self.initial_zones_found and area > AREA_THRESHOLD:
               # corners found are valid and we can find the 'initial zones' 
                zones = map_utils.get_initial_zones(corners, robot_pos)
                self.previous_zones = zones
                self.initial_zones_found = True
            if self.initial_zones_found: 
                print("decude next zones from previous zones")




            # d. make and save the nice figure
            if self.is_saving:
                save_name = "/home/arthur/dev/ros/data/maps/rects/"+self.map_name+str(self.saving_index)+".png"
                map_utils.make_nice_plot(binary, save_name, robot_pos, self.theta, contours, corners, zones)
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

