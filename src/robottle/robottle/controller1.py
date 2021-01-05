import rclpy
from rclpy.node import Node
import numpy as np
import time
import sys

from vision_msgs.msg import Detection2DArray
from interfaces.msg import Map, Position, Status, LidarData
from std_msgs.msg import String

from robottle_utils import map_utils, controller_utils, vision_utils, lidar_utils
from robottle_utils.vizualiser import ImageVizualiser
from robottle_utils.rrt_star import RRTStar

### STATE MACHINES

INITIAL_ROTATION_MODE = "initial_rotation_mode"
TRAVEL_MODE = "travel_mode"
RANDOM_SEARCH_MODE = "random_search_mode"
BOTTLE_PICKING_MODE = "bottle_picking_mode"
BOTTLE_RELEASE_MODE = "bottle_release_mode"
BOTTLE_REACHING_MODE = "bottle_reaching_mode"

TIMER_STATE_OFF = "0"
TIMER_STATE_ON_TRAVEL_MODE = "1"
TIMER_STATE_ON_RANDOM_SEARCH_BOTTLE_ALIGNMENT = "2"
TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION = "3"
TIMER_STATE_ON_BOTTLE_RELEASE = "4"

DETECTNET_ON = "ON"
DETECTNET_OFF = "OFF"

### HYPERPARAMETERS

# min area that a rotated rectangle must contain to be considered as valid
AREA_THRESHOLD = 60000
# distance at which, if the robot is closer than the goal, travel_mode ends
MIN_DIST_TO_GOAL = 50 # [pixels]
# distance to recycling
MIN_DIST_TO_RECYCLING = 5
# min distance between robot and point in the path to consider the robot as passed it
MIN_DIST_TO_POINT = 0.2 # [m]
# time constant of path computation update (the bigger, the less often the path is updated)
CONTROLLER_TIME_CONSTANT = 20
# path-tracker min angle diff for directing the robot
MIN_ANGLE_DIFF = 15 # [deg]
# Array containing indices of zones to visit: note that zones = [r, z2, z3, z4]
# z2 = grass, z3 = rocks
TARGETS_TO_VISIT = [1,0,2,0] # = grass, recycling, rocks, recycling
ROCKS_ZONE_INDEX = 2
# delta degree for little random search rotations
DELTA_RANDOM_SEARCH = 30
# time to wait for detections on each flip of the camera
TIME_FOR_VISION_DETECTION = 2 # [s]
# maximum number of bottles robot can pick
MAX_BOTTLE_PICKED = 5
# maximum number of times controller enters random search mode inside 1 zone
N_RANDOM_SEARCH_MAX = 20

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

        # Create subscription for lidar
        self.subscription_lidar = self.create_subscription(LidarData, 'lidar_data',
            self.lidar_callback, 1000)

        # subscription for debugng
        self.loging_ling_sub = self.create_subscription(String, 'log_line', self.log_line, 5)

        # Create a publication for uart commands
        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)

        # create publisher for controlling the camera
        self.cam_publisher = self.create_publisher(String, 'detectnet/camera_control', 1000)
        self.set_detectnet_state(DETECTNET_OFF)

        # create publisher for flipping camera
        self.camera_flip_topic = self.create_publisher(String, 'video_source/flip_topic', 1000)
        self.camera_flip_topic.publish(String(data="normal"))
        self.is_flipped = False 


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
        self.lidar_save_index = None
        self.n_random_search = 0
        self.bottles_picked = 0
        self.state = INITIAL_ROTATION_MODE
        self.rotation_timer_state = TIMER_STATE_OFF
        self.is_traveling_forward = False
        self.has_to_find_new_path = False

        # DEBUG
        # set saving state (if True, then it will save some maps to a folder when they can be analysed)
        args = sys.argv
        self.args = args
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
        print("Controller is ready: Is Ploting ? {}  - Is Saving ? {} - rate = {}".format(self.is_plotting, self.is_saving, self.SAVE_TIME_CONSTANT))

        # for debugging
        if "--travel" in args:
            self.start_travel_mode()

        if "--search" in args:
            self.state = RANDOM_SEARCH_MODE
            self.start_random_search_detection()

        if "--reach" in args:
            self.state = BOTTLE_REACHING_MODE
            #self.lidar_save_index = 0


        # STATE MACHINE
        # send a request for continuous rotation after waiting 1 second for UART node to be ready
        # todo: change '0' to '3' when launching controller1 within launch file
        time.sleep(0)
        if self.state == INITIAL_ROTATION_MODE:
            self.uart_publisher.publish(String(data = "r"))


    ### CALLBACKS
    # callbacks are the entry points to all other methods

    def listener_callback_map(self, map_message):
        if self.state == TRAVEL_MODE:
            self.travel_mode(map_message)

    def listener_callback_position(self, pos):
        """This function just receives the position and will update it to self variables.
        All control logics are in the 'map' calback"""
        # receive the position from the SLAM
        self.x = pos.x / 1200
        self.y = pos.y / 1200
        self.theta = pos.theta % 360

    def lidar_callback(self, msg):
        if "--travel" in self.args: 
            if self.robot_pos is None or self.zones is None or not len(self.robot_pos) or self.theta is None:
                return
            is_rock, angle = controller_utils.is_obstacle_a_rock(np.concatenate((self.robot_pos, [self.theta]), axis = 0), 
                    self.zones)

        if self.state == BOTTLE_REACHING_MODE:
            obstacle_detected = lidar_utils.check_obstacle_ahead(msg.distances, msg.angles, self.lidar_save_index) 
            if self.lidar_save_index is not None:
                self.lidar_save_index += 1 
            if obstacle_detected: 
                print("Obstacle detected AHEAD of lidar. Let's STOP.")
                self.uart_publisher.publish(String(data="x"))
                self.start_rotation_timer(DELTA_RANDOM_SEARCH, TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION)

        elif self.state == TRAVEL_MODE and self.is_traveling_forward:
            obstacle_detected = lidar_utils.check_obstacle_ahead(msg.distances, msg.angles, self.lidar_save_index) 
            if obstacle_detected:
                print("Obstacle detected AHEAD of lidar. Let's STOP.")
                self.uart_publisher.publish(String(data="x"))
                self.has_to_find_new_path = True

        elif self.state == BOTTLE_RELEASE_MODE and self.is_traveling_forward:
            obstacle_detected = lidar_utils.check_obstacle_ahead(msg.distances, msg.angles, self.lidar_save_index, length_to_check = 700) 
            if obstacle_detected:
                print("Obstacle detected AHEAD of lidar. HOME DETECTED ! ")
                self.uart_publisher.publish(String(data="x"))
                self.uart_publisher.publish(String(data="q"))


    def listener_arduino_status(self, status_msg):
        """Called when Arduino send something to Jetson
        Messages type
        0: ERROR
        1: SUCESS
        2: IN PROGRESS
        """
        status = status_msg.status
        if self.state == INITIAL_ROTATION_MODE:
            if status == 1:
                print("* Initial Rotation Mode --> Travel Mode")
                self.start_travel_mode()

        elif self.state == BOTTLE_REACHING_MODE:
            if status == 0: 
                # = max distance reached
                print("Robot advanced maximum distance in 'y' mode")
                self.start_random_search_detection()
            elif status == 1:
                # = there is a small obstacle ahead of the robot
                if TARGETS_TO_VISIT[self.current_target_index] == ROCKS_ZONE_INDEX: 
                    print("Picking bottle inside rocks")
                    # robot is inside the rocks zone
                    # verify that robot is not in front of the rocks 
                    self.uart_publisher.publish(String(data="x"))
                    is_rock, angle = controller_utils.is_obstacle_a_rock(self.robot_pos, self.zones)
                    if is_rock:
                        # TODO: what the fuck is this
                        self.state = RANDOM_SEARCH_MODE
                        self.start_rotation_timer(angle, TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION)
                    else:
                        self.state = BOTTLE_PICKING_MODE
                        self.uart_publisher.publish(String(data="p"))
                else:
                    print("Picking bottle outside of rocks")
                    self.state = BOTTLE_PICKING_MODE
                    self.uart_publisher.publish(String(data="p"))


        elif self.state == BOTTLE_PICKING_MODE:
            if status == 0:
                # = no bottle were detected by the robot arm 
                self.start_random_search_detection()
            elif status == 1: # robot picked the bottle 
                print("Bottle picked")
                self.bottles_picked += 1
                self.start_random_search_detection()

        elif self.state == BOTTLE_RELEASE_MODE:
            if status == 1:
                self.is_traveling_forward = False
                self.start_travel_mode()

    def listener_callback_detectnet(self, msg):
        """Called when a bottle is detected by neuron network
        This function can only be called when the neuron network is active, 
        i.e. only in RANDOM_SEARCH_MODE when the robot is still and waiting for detection
        """
        # we must verify that the detectnet is really suppose to be turned ON
        if self.detectnet_state == DETECTNET_OFF: 
            return 
        # we must verify that actual flip state is the same as expected flip state
        source_img = msg.detections[0].source_img
        is_actually_flipped = source_img.height < source_img.width
        if is_actually_flipped != self.is_flipped:
            return

        # 1. extract the detection
        print("    Detections successful")
        new_detections = [(d.bbox.center.x, d.bbox.center.y, d.bbox.size_x, d.bbox.size_y, self.is_flipped) for d in msg.detections]
        print(new_detections, msg.detections[0].source_img.width)
        self.detections += new_detections

        # 2. flip the camera
        self.flip_camera_and_reset_detectnet_timer()

    def detection_timer_callback(self):
        """Timer called after 2 seconds and destroyed immeditaly when a bottle is detected.
        If the calback is called, it means no bottle were detected during its period.
        """
        print("    No bottle detected during time interval")
        self.flip_camera_and_reset_detectnet_timer()

    def flip_camera_and_reset_detectnet_timer(self):
        msg = "normal" if self.is_flipped else "flip"
        self.camera_flip_topic.publish(String(data=msg))
        self.destroy_timer(self.wait_for_detectnet_timer)
        self.is_flipped = not self.is_flipped
        if self.is_flipped:
            # = first lap is finished 
            # create a callback in some time to observe bottles around robot
            print("    Trying to detect again with a new flip")
            self.wait_for_detectnet_timer = self.create_timer(TIME_FOR_VISION_DETECTION, self.detection_timer_callback)
        else:
            # = nothing was detected during the second lap
            # get the best bottle to go to
            self.take_bottle_decision()

    def take_bottle_decision(self):
        self.set_detectnet_state(DETECTNET_OFF)
        if len(self.detections):
            # get best detection
            detection = vision_utils.get_best_detections(self.detections)
            # move to bottle
            angle = vision_utils.get_angle_of_detection(detection)
            print("starting timer after detection of bottle, with angle:",angle)
            self.start_rotation_timer(angle, TIMER_STATE_ON_RANDOM_SEARCH_BOTTLE_ALIGNMENT)
        else:
            # lets start a rotation of 30 degrees again
            print("    No bottle detected at all --> start again a rotation")
            self.start_rotation_timer(DELTA_RANDOM_SEARCH, TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION)

    def rotation_timer_callback(self):
        """Called when robot has turned enough to pick the bottle"""
        self.destroy_timer(self.rotation_timer)

        if self.rotation_timer_state == TIMER_STATE_ON_RANDOM_SEARCH_BOTTLE_ALIGNMENT:
            print("    Robot is in front of bottle")
            # change timer state and go to bottle picking mode.
            self.rotation_timer_state = TIMER_STATE_OFF
            self.start_bottle_reaching_mode()

        if self.rotation_timer_state == TIMER_STATE_ON_RANDOM_SEARCH_DELTA_ROTATION:
            print("    Robot delta rotation finished")
            self.rotation_timer_state = TIMER_STATE_OFF
            self.uart_publisher.publish(String(data="x"))
            # start detection again
            self.start_random_search_detection()

        if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE:
            # change timer state and start moving forward.
            self.rotation_timer_state = TIMER_STATE_OFF
            print("    Rotated time reached. Let's move forward.")
            self.uart_publisher.publish(String(data="w"))

        if self.rotation_timer_state == TIMER_STATE_ON_BOTTLE_RELEASE:
            # = robot is aligned with the recycling area
            print("Ready to move forward")
            self.is_traveling_forward = True
            self.uart_publisher.publish(String(data="m2"))
            self.uart_publisher.publish(String(data="w"))
            pass


    ### STATE MACHINE METHODS

    def set_detectnet_state(self, new_state):
        self.detectnet_state = new_state
        if new_state == DETECTNET_ON:
            self.cam_publisher.publish(String(data="create"))
        if new_state == DETECTNET_OFF:
            self.cam_publisher.publish(String(data="destroy"))

    def start_random_search_detection(self):
        """Will start the random search and increase by 1 the stepper
        """
        print("* Random search activated, n = ", self.n_random_search)
        self.state = RANDOM_SEARCH_MODE
        self.n_random_search += 1

        # ending criterion 
        has_to_stop_search = (self.n_random_search == N_RANDOM_SEARCH_MAX) or (self.bottles_picked == MAX_BOTTLE_PICKED)
        if has_to_stop_search:
            print("Leaving random search")
            # no more random walk can happen
            # let's enter travel mode again
            self.set_detectnet_state(DETECTNET_OFF)
            self.start_travel_mode()
            return

        # set lower speed
        self.uart_publisher.publish(String(data = "xm2"))

        # create subscription for detection
        self.set_detectnet_state(DETECTNET_ON)

        # create a callback in some time to observe bottles around robot
        self.detections = []
        self.wait_for_detectnet_timer = self.create_timer(TIME_FOR_VISION_DETECTION, self.detection_timer_callback)

    def travel_mode(self, map_message):
        """Travel mode of the controller.
        This function is called by the map listener's callback.
        """
        # compute robot position (used a lot)
        self.robot_pos = map_utils.pos_to_gridpos(self.x, self.y)

        ### I. Path planning
        # Once in a while, start the path planning logic
        if int(map_message.index) % CONTROLLER_TIME_CONSTANT == 0 or self.has_to_find_new_path:
            print("    map analysis", int(map_message.index))

            ## Handling timer problem
            if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE:
                print("Stopping current timer and let's compute a new path to follow")
                self.uart_publisher.publish(String(data = "x"))
                self.rotation_timer_state = TIMER_STATE_OFF
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
                if area > 240000:
                    raise RuntimeError("Zones were not found properly")

               # corners found are valid and we can find the 'initial zones'
                self.zones = map_utils.get_initial_zones(corners, self.robot_pos)
                self.initial_zones_found = True
                print("    - initial zones found with area: ", area)

            if self.initial_zones_found:
                # update zones with new map
                new_zones = map_utils.get_zones_from_previous(corners, self.zones)
                self.zones = new_zones

                ## Path Planing
                # d. get targets positions for each zones
                self.targets = map_utils.get_targets_from_zones(np.array(self.zones), target_weight = 0.6)

                # e. rrt_star path planning
                self.goal = self.targets[TARGETS_TO_VISIT[self.current_target_index]]
                random_area = map_utils.get_random_area(self.zones)
                print("    - will find path")
                rrt = RRTStar(start = self.robot_pos, goal = self.goal, binary_obstacle = binary, 
                        rand_area = random_area, expand_dis = 50, path_resolution = 1,
                        goal_sample_rate = 5, max_iter = 500)
                self.path = np.array(rrt.planning(animation = False))
                self.has_to_find_new_path = False
                print("    - path found")

        # (make and save the nice figure)
        if (self.is_saving or self.is_plotting) and int(map_message.index) % self.SAVE_TIME_CONSTANT == 0:
            name = self.map_name+str(self.saving_index)
            save_name = "/home/arthur/dev/ros/data/maps/rects/"+name+".png" if self.is_saving else ""
            text = ""
            try:
                img = map_utils.make_nice_plot(self.binary, save_name, self.robot_pos,
                        self.theta, self.contours, self.corners,
                        self.zones, self.targets, self.path.astype(int),
                        text = text)
                if self.is_plotting:
                    self.live_vizualiser.display(np.array(img))
                print("-----> saving index: ", self.saving_index, int(map_message.index))
                self.saving_index += 1
            except:
                print("Could not save")


        ### II. Path Tracking
        # 0. end condition
        if len(self.path) == 0 or self.goal is None: 
            print("...")
            return

        # 1. state transition condition
        dist = controller_utils.get_distance(self.robot_pos, self.goal)
        reached = TARGETS_TO_VISIT[self.current_target_index]
        min_dist = MIN_DIST_TO_RECYCLING if reached == 0 else MIN_DIST_TO_GOAL
        if dist < min_dist:
            # robot arrived to destination
            print("Robot reached zone ", reached)
            self.current_target_index += 1
            self.is_traveling_forward = False
            self.path = []
            if reached in [1,2]: # robot in zone 2 or zone 3
                # travel_mode --> random_search mode
                self.start_random_search_detection()
            elif reached == 0:
                # travel_mode --> release_bottle_mode
                self.start_bottle_release_mode()
            return

        # 2. Else, compute motors commands
        if self.rotation_timer_state == TIMER_STATE_ON_TRAVEL_MODE: 
            return

        path_orientation = controller_utils.get_path_orientation(self.path)
        diff = controller_utils.angle_diff(path_orientation, self.theta)

        if abs(diff) > MIN_ANGLE_DIFF:
            ## ROTATION CORRECTION SUB-STATE
            print("    rotation correction with diff = ", diff)
            self.is_traveling_forward = False
            self.start_rotation_timer(diff, TIMER_STATE_ON_TRAVEL_MODE)

        else:
            ## FORWARD SUB-STATE
            # in theory, robot should be going forward.
            # send a forward message just in case it wasn't lunched before
            self.uart_publisher.publish(String(data = "w"))
            self.is_traveling_forward = True
            # compute distance to next point of the path
            p = self.path[-2]
            dist_to_next_point = controller_utils.get_distance(self.robot_pos, p)
            print("    going foward for a distance {:.2f}, diff = {:.2f}".format(dist_to_next_point, diff))
            if dist_to_next_point < MIN_DIST_TO_POINT:
                # remove first point of the path
                print("Will update path: ", self.path)
                del self.path[-1]
                print("Updated path: ", self.path)

    def start_bottle_reaching_mode(self):
        """Will start the bottle picking mode"""
        self.state = BOTTLE_REACHING_MODE
        self.uart_publisher.publish(String(data = "y"))

    def start_travel_mode(self):
        self.uart_publisher.publish(String(data = "m1"))
        self.has_to_find_new_path = True
        self.state = TRAVEL_MODE

    def start_bottle_release_mode(self):
        """Will start the bottle picking mode"""
        self.state = BOTTLE_RELEASE_MODE
        # 1. get angle to rotate to align robot to correct position
        diagonal_orientation = controller_utils.get_path_orientation([self.zones[0], self.zones[3]])
        angle = controller_utils.angle_diff(diagonal_orientation, self.theta)
        print("BOTTLE RELEASE with angle diff: ", angle, "values ", self.theta, diagonal_orientation)
        # 2. make the rotation
        self.is_traveling_forward = False
        self.start_rotation_timer(angle, TIMER_STATE_ON_BOTTLE_RELEASE)

    ### HELPER FUNCTIONS

    def start_rotation_timer(self, angle, state):
        """Will start a timer which has a period equals to the required rotation time
        to achieve the provided angle."""
        # 1. if required, delete previous timer
        if self.rotation_timer_state is not TIMER_STATE_OFF:
            # it means another timer was launched
            self.destroy_timer(self.rotation_timer)

        # 2. send the rotation motor control
        print("        (starting rotation now)", state, angle)
        msg = String()
        if angle > 0: msg.data = "d"
        else: msg.data = "a"
        self.uart_publisher.publish(msg)

        # 3. estimate remaining time of rotation and start new timer
        self.rotation_timer_state = state
        time_to_rotate = controller_utils.get_rotation_time(np.abs(angle))
        self.rotation_timer = self.create_timer(time_to_rotate, self.rotation_timer_callback)

    def log_line(self, msg):
        print("---------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = Controller1()
    rclpy.spin(node)
    print("Leaving code !")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
