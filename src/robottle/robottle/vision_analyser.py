import rclpy
from rclpy.node import Node

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

    def find_map_corner(self, request, response):
        print("Service received !: ", request)
        # 1. take a picture


        if request.should_save:
            name = strftime("%m-%d_%H-%M-%S", gmtime())
            vision_utils.take_picture(save = True, name = name)


        response.response = "I have saved your picture"
        print("Response: ", response)
        return response


def main(args=None):
    rclpy.init(args=args)
    vision_analyser = VisionAnalyser()
    rclpy.spin(vision_analyser)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
