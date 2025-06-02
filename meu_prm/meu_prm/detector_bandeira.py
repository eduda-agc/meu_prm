import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorBandeira(Node):
    def __init__(self):
        super().__init__('detector_bandeira')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Bool, 'bandeira_detectada', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #faixa de vermelho no HSV 
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 | mask2

        area = cv2.countNonZero(mask)

        detectado = area > 500  
        self.pub.publish(Bool(data=detectado))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorBandeira()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
