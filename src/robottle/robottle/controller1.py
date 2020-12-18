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

### STATE MACHINES

INITIAL_ROTATION_MODE = "initial_rotation_mode"
TRAVEL_MODE = "travel_mode"
RANDOM_SEARCH_MODE = "random_search_mode"
BOTTLE_PICKING_MODE = "bottle_picking_mode"
BOTTLE_RELEASE_MODE = "bottle_release_mode"

TIMER_STATE_OFF = "0"
TIMER_STATE_ON_TRAVEL_MODE = "1"
TIMER_STATE_ON_RANDOM_SEARCH_BOTTLE_ALIGNMENT = "2"
TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION = "3"

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
# maximum number of times controller enters random search mode inside 1 zone
N_RANDOM_SEARCH_MAX = 5
# Array containing indices of zones to visit: note that zones = [r, z2, z3, z4]
TARGETS_TO_VISIT = [2,0,1,0]
# delta degree for little random search rotations
DELTA_RANDOM_SEARCH = 30

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
        # CHANGED: c'est normal que tu destroy juste apres crée la subscription?

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
        self.n_random_search = 0
        self.state = INITIAL_ROTATION_MODE
        self.rotation_timer_state = TIMER_STATE_OFF

        # DEBUG
        # set saving state (if True, then it will save some maps to a folder when they can be analysed)
        args = sys.argv
        self.is_saving = "--save" in args
        self.is_plotting = "--plot" in args
        self.saving_index = 0
        self.map_name = ""
        self.SAVE_TIME_CONSTANT = 10
        if self.is_plotting:
            self.live_vizualiser = ImageVizualiser()
            try:
                self.SAVE_TIME_CONSTANT = int(args[args.index("--plot")+1])
            except:
                pass
        if self.is_saving:
            idx = args.index("--save")
            self.map_name = args[idx + 1]
            print("Name : ", self.map_name)
            try:
                self.SAVE_TIME_CONSTANT = int(args[idx+2])
            except:
                pass

        # for debugging
        if "--travel" in args:
            self.state = TRAVEL_MODE

        if "--search" in args:
            # CHANGED
            print("Random search mode activated")
            self.state = RANDOM_SEARCH_MODE
            self.start_random_search_mode()


        # STATE MACHINE
        # send a request for continuous rotation after waiting 1 second for UART node to be ready
        # todo: change '0' to '3' when launching controller1 within launch file
        time.sleep(0)
        if self.state == INITIAL_ROTATION_MODE:
            self.uart_publisher.publish(String(data = "r"))
        print("Controller is ready: Is Ploting ? {}  - Is Saving ? {} - rate = {}".format(self.is_plotting, self.is_saving, self.SAVE_TIME_CONSTANT))


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
        self.x = pos.x / 1200
        self.y = pos.y / 1200
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
            if status == 0:
                # ERROR --> we must do something
                # todo !!!
                pass
            elif status == 1: # SUCCESS --> bottle was probably picked
                self.start_random_search_mode()

        if self.state == BOTTLE_RELEASE_MODE:
            if status == 1:
                self.state = TRAVEL_MODE

    def listener_callback_detectnet(self, msg):
        """Called when a bottle is detected by neuron network
        This function can only be called when the neuron network is active
        i.e. only in RANDOM_SEARCH_MODE when the robot is still and waiting for detection
        """
        if self.rotation_timer_state == TIMER_STATE_OFF:
            # 1. destroy the timer 
            self.destroy_timer(self.wait_for_detectnet_timer)
            # 2. find the angle of the closest detected bottle
            detections = [(d.bbox.center.x, d.bbox.center.y, d.bbox.size_x, d.bbox.size_y) for d in msg.detections]
            angle = vision_utils.get_angle_of_closest_bottle(detections)
            # 3. rotation timer
            if angle is not None:
                print("starting timer after detection of bottle, with angle:",angle)
                self.cam_publisher.publish(String(data="destroy"))
                self.start_rotation_timer(angle, TIMER_STATE_ON_RANDOM_SEARCH_BOTTLE_ALIGNMENT)

    def rotation_timer_callback(self):
        """Called when robot has turned enough to pick the bottle"""
        self.destroy_timer(self.rotation_timer)

        if self.rotation_timer_state == TIMER_STATE_ON_RANDOM_SEARCH_BOTTLE_ALIGNMENT:
            print("Robot is in front of bottle")
            # change timer state and go to bottle picking mode.
            self.rotation_timer_state = TIMER_STATE_OFF
            self.start_bottle_picking_mode()

        if self.rotation_timer_state == TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION:
            print("Robot finished his delta rotation")
            self.rotation_timer_state = TIMER_STATE_OFF
            # start detection again
            self.start_random_search_mode()

        if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE:
            # change timer state and start moving forward.
            self.rotation_timer_state = TIMER_STATE_OFF
            print("Rotated time reached. Let's move forward.")
            self.uart_publisher.publish(String(data="w"))

    def no_bottle_detected_callback(self):
        """Timer called after 2 seconds and destroyed immeditaly when a bottle is detected.
        If the calback is called, it means no bottle were detected during its period.
        """
        print("No bottle detected during time interval")
        self.destroy_timer(self.wait_for_detectnet_timer)
        # lets start a rotation
        self.cam_publisher.publish(String(data="destroy"))
        self.start_rotation_timer(DELTA_RANDOM_SEARCH, TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION)

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
                self.rotation_timer_state == TIMER_STATE_OFF
                self.destroy_timer(self.rotation_timer)

            ## Map analysis
            # a. filter the map
            map_data = bytearray(map_message.map_data)
            m = map_utils.get_map(map_data)
            binary = map_utils.filter_map(m, dilation_kernel_size = 14)

            # b. get rectangle around the map
            try:
                corners, area, contours = map_utils.get_bounding_rect(binary)
            except:
                print("Contours not found... yet ?")
                return

            # save binary if we are going to make some plots
            if self.is_saving or self.is_plotting:
                self.binary = binary
                self.contours = contours
                self.corners = corners

            # c. find zones
            # zones are ordered the following way: (recycling area, zone2, zone3, zone4)
            if not self.initial_zones_found and area > AREA_THRESHOLD:
                if area > 2250000:
                    raise RuntimeError("Zones were not found properly")

               # corners found are valid and we can find the 'initial zones'
                self.zones = map_utils.get_initial_zones(corners, self.robot_pos)
                self.initial_zones_found = True
                print("Initial zones found with area: ", area)

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
        if (self.is_saving or self.is_plotting) and int(map_message.index) % self.SAVE_TIME_CONSTANT == 0:
            name = self.map_name+str(self.saving_index)
            save_name = "/home/arthur/dev/ros/data/maps/rects/"+name+".png" if self.is_saving else ""
            text = "robot pos = {}".format(int(map_message.index),
                    (self.robot_pos, self.theta))
            try:
                img = map_utils.make_nice_plot(self.binary, save_name, self.robot_pos,
                        self.theta, self.contours, self.corners,
                        self.zones, self.targets, self.path.astype(int),
                        text = text)
                if self.is_plotting:
                    self.live_vizualiser.display(np.array(img))
                # np.save("/home/arthur/dev/ros/data/maps/"+name+".npy", img)
                self.saving_index += 1
            except:
                print("Could not save")

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
        print("entered random search mode", self.n_random_search+1,"times")
        self.state = RANDOM_SEARCH_MODE
        self.n_random_search += 1

        # ending criterion (1)
        if self.n_random_search == N_RANDOM_SEARCH_MAX:
            # no more random walk can happen
            # let's enter travel mode again
            self.cam_publisher.publish(String(data="destroy"))
            self.state = TRAVEL_MODE
            return

        # create subscription for detection
        self.cam_publisher.publish(String(data="create"))

        # create a callback in 2 seconds in the case that no bottle were detected
        self.wait_for_detectnet_timer = self.create_timer(3, self.no_bottle_detected_callback)


    def start_bottle_picking_mode(self):
        """Will start the bottle picking mode"""
        self.state = BOTTLE_PICKING_MODE
        # would be nice to go slower here
        self.uart_publisher.publish(String(data = "y"))

    def start_bottle_release_mode(self):
        """Will start the bottle picking mode"""
        self.state = BOTTLE_RELEASE_MODE
        # todo !!!

    ### HELPER FUNCTIONS

    def start_rotation_timer(self, angle, state):
        """Will start a timer which has a period equals to the required rotation time
        to achieve the provided angle."""
        # 1. if required, delete previous timer
        if self.rotation_timer_state is not TIMER_STATE_OFF:
            # it means another timer was launched
            self.destroy_timer(self.rotation_timer)

        # 2. send the rotation motor control
        print("starting rotation now")
        msg = String()
        if angle > 0: msg.data = "d"
        else: msg.data = "a"
        self.uart_publisher.publish(msg)

        # 3. estimate remaining time of rotation and start new timer
        self.rotation_timer_state = state
        time_to_rotate = controller_utils.get_rotation_time(np.abs(angle))
        self.rotation_timer = self.create_timer(time_to_rotate, self.rotation_timer_callback)

def main(args=None):
    rclpy.init(args=args)
    node = Controller1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
