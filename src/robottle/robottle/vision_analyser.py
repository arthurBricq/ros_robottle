import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from interfaces.srv import FindMapCorner

from time import gmtime, strftime
from robottle_utils import vision_utils

class VisionAnalyser(Node):
    """Ros Node to
    - take a picture at the moment a service is received
    - perform image analysis
    - returns the output of the algorithm (beacon delta_angles and color)
    Eventually it can also save images to a folder.
    """

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FindMapCorner, 'find_map_corner', self.find_map_corner)

        self.pictures_to_take = 0
        self.has_received_detection = False

        # 1. create subscription to camera topic
        self.subscription = self.create_subscription(Image, 'detectnet/overlay',
                self.raw_image_callback, 1000)
        self.subscription = self.create_subscription(Image, 'detectnet/detections',
                self.detection_callback, 1000)

    def find_map_corner(self, request, response):
        print("Service received !: ", request)
        self.pictures_to_take += 1
        response.response = "I have  asked to save your picture"
        return response

    def raw_image_callback(self, msg):
        """Called when an image is received from 'video_source/raw'"""
        if self.pictures_to_take:
            if self.has_received_detection:
                self.pictures_to_take -= 1
                # so let's analyse it here and then delete the subscription
                rows = msg.height
                step = msg.step
                cols = msg.width
                dim = int(step / cols)
                pixels = msg.data # of size (steps, nrows)
                name = strftime("%m-%d_%H-%M-%S", gmtime())
                # save the image (later we will need to analyse it)
                vision_utils.save_picture(pixels, rows, cols, dim, name)

    def detection_callback(self, msg):
        if self.pictures_to_take:
            print("Detection message: ", msg)



def main(args=None):
    rclpy.init(args=args)
    vision_analyser = VisionAnalyser()
    rclpy.spin(vision_analyser)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
