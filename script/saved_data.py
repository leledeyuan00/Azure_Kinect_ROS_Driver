import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import cv2
import os
import datetime

# import cv bridge
from cv_bridge import CvBridge

class SavedImages(Node):
    def __init__(self):
        super().__init__('saved_images')
        self.image = None
        self.subscription = self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a service for saving images
        self.srv = self.create_service(Trigger, 'save_image', self.save_image_callback)

        # Create a folder for saving data
        self.home_dir = os.path.expanduser('~')
        self.folder_name = self.home_dir + '/Documents/logs/datasets/'
        self.saved_data_init()
    
    def saved_data_init(self):
        # Create a folder for saving data by using system command 'mkdir'
        os.system('mkdir -p ' + self.folder_name)

    def listener_callback(self, msg):
        self.image = msg

    def save_image_callback(self, request, response):
        
        # From ros msg to cv2 image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')

        # Get current system time for file name
        now = datetime.datetime.now()
        time_stamp = now.strftime("%m-%d-%H-%M-%S")
        file_name = self.folder_name + time_stamp + '.png'

        # Save the image
        if cv2.imwrite(file_name, cv_image):
            self.get_logger().info('Saved image to ' + file_name)
        else:
            self.get_logger().info('Failed to save image to ' + file_name)
        
        # Return the response message
        response.success = True
        response.message = "Image saved to " + file_name + " successfully."
        return response


def main(args=None):
    rclpy.init(args=args)

    save_images = SavedImages()

    rclpy.spin(save_images)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_images.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()