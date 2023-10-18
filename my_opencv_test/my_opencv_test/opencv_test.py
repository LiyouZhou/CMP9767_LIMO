import rclpy
from rclpy.node import Node
from cv2 import namedWindow, cvtColor, imshow, inRange

from cv2 import destroyAllWindows, startWindowThread, bitwise_and
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny, resize, INTER_CUBIC
import cv2
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ImageConverter(Node):

    def __init__(self):
        super().__init__('opencv_test')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image,
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    10)
        self.canny_image_publisher = self.create_publisher(Image,
                                                    "/limo/depth_camera_link/image_canny",
                                                    10)
        self.masked_image_publisher = self.create_publisher(Image,
                                                    "/limo/depth_camera_link/image_masked",
                                                    10)
        self.mask_image_publisher = self.create_publisher(Image,
                                                    "/limo/depth_camera_link/image_mask",
                                                    10)
    def image_callback(self, data):
        # namedWindow("Image window")
        # namedWindow("masked")
        # namedWindow("canny")

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)

        mask = inRange(cv_image, (150, 150, 150), (255, 255, 255))

        print(np.sum(mask))

        masked_img = bitwise_and(cv_image, cv_image, mask = mask)
        gray_img = cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        canny_img = Canny(gray_img, 10, 200)

        mask_img = cvtColor(mask, cv2.COLOR_GRAY2RGB)


        self.canny_image_publisher.publish(self.bridge.cv2_to_imgmsg(canny_img, encoding="passthrough"))
        self.masked_image_publisher.publish(self.bridge.cv2_to_imgmsg(masked_img, encoding="rgb8"))
        self.mask_image_publisher.publish(self.bridge.cv2_to_imgmsg(mask_img, encoding="rgb8"))

        # imshow("masked", mask)
        # imshow("canny", img3)
        # imshow("Image window", cv_image)
        waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
