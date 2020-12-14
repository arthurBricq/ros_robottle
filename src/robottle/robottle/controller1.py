import rclpy
from rclpy.node import Node
import numpy as np 
import time
import sys

from vision_msgs.msg import Detection2DArray
from interfaces.msg import Map, Position, Status
from std_msgs.msg import String

from robottle_utils import map_utils, controller_utils, vision_utils
from robottle_utils.vizualiser import ImageVizualiser
from robottle_utils.rrt_star import RRTStar

### STATE MACHINE 

INITIAL_ROTATION_MODE = "initial_rotation_mode"
TRAVEL_MODE = "travel_mode"
RANDOM_SEARCH_MODE = "random_search_mode"
BOTTLE_PICKING_MODE = "bottle_picking_mode"
BOTTLE_RELEASE_MODE = "bottle_release_mode"

TIME_STATE_OFF = "0"
TIMER_STATE_ON_TRAVEL_MODE = "1"
TIMER_STATE_ON_RANDOM_SEARCH = "2"

### HYPERPARAMETERS

# min area that a rotated rectangle must contain to be considered as valid
AREA_THRESHOLD = 60000
# distance at which, if the robot is closer than the goal, travel_mode ends
MIN_DIST_TO_GOAL = 1 # [m]
# min distance between robot and point in the path to consider the robot as passed it
MIN_DIST_TO_POINT = 0.2 # [m]
# time constant of path computation update (the bigger, the less often the path is updated)
CONTROLLER_TIME_CONSTANT = 20
# path-tracker min angle diff for directing the robot
MIN_ANGLE_DIFF = 15 # [deg]
# time taken to do a 360 degrees rotation
INITIAL_ROTATION_TIME = 10 # [s]
# maximum number of times controller enters random search mode inside 1 zone
N_RANDOM_SEARCH_MAX = 5
# Array containing indices of zones to visit: note that zones = [r, z2, z3, z4]
TARGETS_TO_VISIT = [2,0,1,0]

class Controller1(Node):
    """
    Controller of the ROBOT
    This node is a state machine with several states and transitions from one to each others.
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

        # Create subscription for detectnet
        self.subscription_camera = self.create_subscription(Detection2DArray, '/detectnet/detections',
            self.listener_callback_detectnet, 1000)
        
        # Create a publication for uart commands
        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)
        
        # create publisher for controlling the camera
        self.cam_publisher = self.create_publisher(String, 'detectnet/camera_control', 1000)
        self.cam_publisher.publish(String(data="destroy"))

        # keep track of where is the robot within the class
        self.x = 0
        self.y = 0
        self.theta = 0 
        
        # variable for the controller
        self.initial_zones_found = False
        self.zones = np.array([])
        self.path = np.array([])
        self.targets = []
        self.goal = None
        self.robot_pos = None
        self.current_target_index = 0
        self.rotation_timer = None

        # DEBUG 
        # set saving state (if True, then it will save some maps to a folder when they can be analysed)
        args = sys.argv
        self.is_saving = "--save" in args 
        self.is_plotting = "--plot" in args
        self.saving_index = 0
        if self.is_plotting:
            self.map_name = ""
            self.live_vizualiser = ImageVizualiser()
        if self.is_saving: 
            self.map_name = args[args.index("--save") + 1]
            print("Name : ", self.map_name)

        # STATE MACHINE
        # send a request for continuous rotation after waiting 1 second for UART node to be ready
        self.state = INITIAL_ROTATION_MODE
        self.rotation_timer_state = TIME_STATE_OFF
        time.sleep(0)
        self.uart_publisher.publish(String(data = "r"))
        print("Controller is ready: Is Ploting ? {}  - Is Saving ? {}".format(self.is_plotting, self.is_saving))

        # for debugging
        if "--travel" in args: 
            self.state = TRAVEL_MODE
        

    ### CALLBACKS
    # callbacks are the entry points to all other methods

    def listener_callback_map(self, map_message):
        #print("{} - theta = {}".format(int(map_message.index), self.theta))
        if self.state == TRAVEL_MODE: 
            self.travel_mode(map_message)

    def listener_callback_position(self, pos):
        """This function just receives the position and will update it to self variables. 
        All control logics are in the 'map' calback"""
        # receive the position from the SLAM
        self.x = pos.x / 1000
        self.y = pos.y / 1000
        self.theta = pos.theta % 360

    def listener_arduino_status(self, status_msg):
        """Called when Arduino send something to Jetson
        Messages type
        0: ERROR
        1: SUCESS
        2: IN PROGRESS
        """
        status = status_msg.status
        # state machine logic
        if self.state == INITIAL_ROTATION_MODE:
            if status == 1:
                print("Initial rotation mode is finished")
                self.state = TRAVEL_MODE

        if self.state == BOTTLE_PICKING_MODE:
            if status == 0: # ERROR --> we must do something
                # todo !!! 
                pass 
            elif status == 1: # SUCCESS --> bottle was probably picked
                self.start_random_search_mode()

        if self.state == BOTTLE_RELEASE_MODE:
            if status == 1: 
                self.state = TRAVEL_MODE

    def listener_callback_detectnet(self, msg):
        """Called when a bottle is detected by neuron network
        """
        if self.state == RANDOM_SEARCH_MODE:
            # find the angle of the closest detected bottle
            angle = vision_utils.get_angle_of_closest_bottle([])
            # rotation timer
            self.start_rotation_timer(diff, TIMER_STATE_ON_RANDOM_SEARCH)

        if self.state == BOTTLE_PICKING_MODE:
            # look if the bottle in range
            is_bottle_in_range = False
            if is_bottle_in_range:
                # try to catch it - and wait for a response
                self.uart_publisher.publish(String(data = "x"))
                self.uart_publisher.publish(String(data = "p"))

    def rotation_timer_callback(self):
        """Called when robot has turned enough to pick the bottle"""
        self.destroy_timer(self.rotation_timer)

        if self.rotation_timer_state == TIMER_STATE_ON_RANDOM_SEARCH:
            # change timer state and go to bottle picking mode.
            self.rotation_timer_state = TIME_STATE_OFF
            self.start_bottle_picking_mode()

        if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE:
            # change timer state and start moving forward.
            self.rotation_timer_state = TIME_STATE_OFF
            print("Rotated time reached. Let's move forward.")
            self.uart_publisher.publish(String(data="w"))


    ### STATE MACHINE METHODS

    def travel_mode(self, map_message):
        """Travel mode of the controller. 
        This function is called by the map listener's callback.
        """
        # compute robot position (used a lot)
        self.robot_pos = map_utils.pos_to_gridpos(self.x, self.y)

        ### I. Path planning
        # Once in a while, start the path planning logic
        if int(map_message.index) % CONTROLLER_TIME_CONSTANT == 0: 
            print("Starting map analysis")

            ## Handling timer problem
            if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE:
                self.uart_publisher.publish(String(data = "x"))
                self.rotation_timer_state == TIME_STATE_OFF
                self.destroy_timer(self.rotation_timer)

            ## Map analysis 
            # a. filter the map 
            map_data = bytearray(map_message.map_data)
            m = map_utils.get_map(map_data)
            binary = map_utils.filter_map(m, dilation_kernel_size = 14)

            # b. get rectangle around the map
            corners, area, contours = map_utils.get_bounding_rect(binary)

            # save binary if we are going to make some plots
            if self.is_saving or self.is_plotting:
                self.binary = binary
                self.contours = contours 
                self.corners = corners

            # c. find zones 
            # zones are ordered the following way: (recycling area, zone2, zone3, zone4)
            if not self.initial_zones_found and area > AREA_THRESHOLD:
                if area > 225000:
                    raise RuntimeError("Zones were not found properly")

               # corners found are valid and we can find the 'initial zones' 
                self.zones = map_utils.get_initial_zones(corners, self.robot_pos)
                self.initial_zones_found = True
                print("Initial zones found")

            if self.initial_zones_found: 
                # update zones with new map
                new_zones = map_utils.get_zones_from_previous(corners, self.zones)
                self.zones = new_zones

                ## Path Planing
                # d. get targets positions for each zones
                self.targets = map_utils.get_targets_from_zones(np.array(self.zones), target_weight = 0.6)

                # e. rrt_star path planning
                self.goal = self.targets[TARGETS_TO_VISIT[self.current_target_index]]
                rrt = RRTStar(start = self.robot_pos, goal = self.goal,binary_obstacle = binary,rand_area = [0, 500],
                        expand_dis = 50, path_resolution = 1,goal_sample_rate = 5,max_iter = 500)
                self.path = np.array(rrt.planning(animation = False))
                print("Path found")

        # finally. make and save the nice figure
        if (self.is_saving or self.is_plotting) and int(map_message.index) % 5 == 0:
            name = self.map_name+str(self.saving_index)
            save_name = "/home/arthur/dev/ros/data/maps/rects/"+name+".png" if self.is_saving else ""
            text = "robot pos = {}".format(int(map_message.index), 
                    (self.robot_pos, self.theta))
            img = map_utils.make_nice_plot(self.binary, save_name, self.robot_pos, 
                    self.theta, self.contours, self.corners, 
                    self.zones, self.targets, self.path.astype(int), 
                    text = text)
            if self.is_plotting:
                self.live_vizualiser.display(np.array(img))
            # np.save("/home/arthur/dev/ros/data/maps/"+name+".npy", img)
            self.saving_index += 1

        ### II. Path Tracking
        # 0. end condition
        if len(self.path) == 0 or self.goal is None: return 
        if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE: return

        # 1. state transition condition
        dist = controller_utils.get_distance(self.robot_pos, self.goal)
        if dist < MIN_DIST_TO_GOAL:
            print("leaving travel mode")
            # robot arrived to destination
            self.current_target_index += 1
            if self.goal in [1,2]: # robot in zone 2 or zone 3
                # activate camera detection
                self.cam_publisher.publish(String(data="create"))
                # travel_mode --> random_search mode
                self.start_random_search_mode()
            elif self.goal == 0:
                # travel_mode --> release_bottle_mode
                self.start_bottle_release_mode()
            return

        # 2. Else, compute motors commands
        path_orientation = controller_utils.get_path_orientation(self.path)
        diff = (path_orientation - self.theta + 180) % 360 - 180
        if abs(diff) > MIN_ANGLE_DIFF:
            ## ROTATION CORRECTION SUB-STATE
            print("Rotation correction with diff = ", diff)
            # a. send the rotation message
            msg = String()
            if diff > 0: msg.data = "d"
            else: msg.data = "a"
            self.uart_publisher.publish(msg)
            # b. rotation timer
            self.start_rotation_timer(diff, TIMER_STATE_ON_TRAVEL_MODE)

        else:
            ## FORWARD SUB-STATE
            # in theory, robot should be going forward.
            # send a forward message just in case it wasn't lunched before
            self.uart_publisher.publish(String(data = "w"))
            # compute distance to next point of the path
            p = self.path[-2]
            dist_to_next_point = controller_utils.get_distance(self.robot_pos, p)
            print(int(map_message.index), "robot is going forward, distance to next point: ", dist_to_next_point)
            if dist_to_next_point < MIN_DIST_TO_POINT:
                # remove first point of the path
                print("Will update path: ", self.path)
                del self.path[-1]
                print("Updated path: ", self.path)


    def start_random_search_mode(self):
        """Will start the random search and increase by 1 the stepper
        """
        if self.n_random_search == N_RANDOM_SEARCH_MAX:
            # no more random walk can happen
            # let's enter travel mode again
            self.cam_publisher.publish(String(data="destroy"))
            self.state = TRAVEL_MODE
            return 

        self.state = RANDOM_SEARCH_MODE
        self.uart_publisher.publish(String(data = "d"))
        self.n_random_search += 1

    def start_bottle_picking_mode(self):
        """Will start the bottle picking mode"""
        self.state = BOTTLE_PICKING_MODE
        # would be nice to go slower here
        self.uart_publisher.publish(String(data = "w"))

    def start_bottle_release_mode(self):
        """Will start the bottle picking mode"""
        self.state = BOTTLE_RELEASE_MODE
       
        # todo !!! 

    ### HELPER FUNCTIONS

    def start_rotation_timer(self, angle, state):
        """Will start a timer which has a period equals to the required rotation time
        to achieve the provided angle."""
        # 1. if required, delete previous timer
        if self.rotation_timer_state is not TIME_STATE_OFF: 
            # it means another timer was launched
            self.destroy_timer(self.rotation_timer)

        # 2. estimate remaining time of rotation and start new timer
        self.rotation_timer_state = state 
        time_to_rotate = controller_utils.get_rotation_time(np.abs(angle))
        self.rotation_timer = self.rotation_timer = self.create_timer(time_to_rotate, self.rotation_timer_callback)





def main(args=None):
    rclpy.init(args=args)
    node = Controller1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

