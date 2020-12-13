import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from interfaces.srv import FindMapCorner

from time import gmtime, strftime
from robottle_utils import vision_utils

class VisionAnalyser(Node):
    """Ros Node to TEST TIMERS 
    """

    def __init__(self):
        super().__init__('minimal_service')
        self.i = 0
        self.timer1 = self.create_timer(1, self.timer1_callback) 
        self.timer2 = self.create_timer(1, self.timer2_callback) 

    def timer1_callback(self):
        if self.i == 5: 
            self.destroy_timer(self.timer1)
        print("timer 1 calledi: ", self.i)
        self.i += 1
        

    def timer2_callback(self):
        print("timer 2 checking timer 1: ", self.timer1.is_ready())



def main(args=None):
    rclpy.init(args=args)
    vision_analyser = VisionAnalyser()
    rclpy.spin(vision_analyser)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
