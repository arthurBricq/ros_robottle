import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from interfaces.srv import FindMapCorner

from time import gmtime, strftime
from robottle_utils import vision_utils

FOLDER = "/home/arthur/dev/ros/pictures/f1/"

class VisionAnalyser(Node):
    """Ros Node to
    - take a picture at the moment a service is received
    - perform image analysis
    - returns the output of the algorithm (beacon delta_angles and color)
    Eventually it can also save images to a folder.
    """

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FindMapCorner, 'find_map_corner', self.vision_service)

        # parameters
        self.pictures_to_take = 0
        self.detection_to_receive = 0

        # subscriptions
        self.subscription = self.create_subscription(Image, 'detectnet/overlay',# 'video_source/raw',
                self.raw_image_callback, 1000)
        self.subscription = self.create_subscription(Detection2DArray, 'detectnet/detections',
                self.detection_callback, 1000)

    ### SERVICE
    def vision_service(self, request, response):
        print("Service received !: ", request)
        self.pictures_to_take += 1
        self.detection_to_receive += 1
        self.name = request.name 
        response.response = "I have  asked to save your picture at : {}".format(self.name)
        return response

    ### SUBSCRIPTIONS
    def raw_image_callback(self, msg):
        """Called when an image is received from 'video_source/raw'
        It will simply save the first image that comes once a detection was made.
        """
        if self.pictures_to_take and not self.detection_to_receive:
            self.pictures_to_take -= 1
            # so let's analyse it here and then delete the subscription
            rows = msg.height
            step = msg.step
            cols = msg.width
            dim = int(step / cols)
            pixels = msg.data # of size (steps, nrows)
            # save the image (later we will need to analyse it)
            # vision_utils.save_picture(pixels, rows, cols, dim, self.name, FOLDER)

    def detection_callback(self, msg):
        detections = [(d.bbox.center.x, d.bbox.center.y, d.bbox.size_x, d.bbox.size_y) for d in msg.detections]
        angle = vision_utils.get_angle_of_closest_bottle(detections)
        print("Angle = ", angle)
#        if self.detection_to_receive:
            #self.detection_to_receive -= 1
            #res = [(d.bbox.center.x, d.bbox.center.y, d.bbox.size_x, d.bbox.size_y) for d in msg.detections]
            #with open(FOLDER + "_detection.txt", "a") as text_file:
            #    string = "\n - " + self.name + str(res)
            #    text_file.write(string)



def main(args=None):
    rclpy.init(args=args)
    vision_analyser = VisionAnalyser()
    rclpy.spin(vision_analyser)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
