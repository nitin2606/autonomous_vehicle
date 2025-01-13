
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cap = cv2.VideoCapture(0)  # Open the first camera
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.publish_frame)  # 30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    rclpy.spin(node)
    rclpy.shutdown()
