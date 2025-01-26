import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .yolo_inference import YoloInference

DEFAULT_TOPIC_CAM_0 = 'cam_0/frame'
DEFAULT_MODL_PATH = '/home/knight/autonomous_vehicle_ws/src/model_files/last_single_class_new.pt'

callback_counter = 0

class FrameSubscriber(Node):
    def __init__(self):
        super().__init__('frame_subscriber')

        self.declare_parameter('cam_0.topic', DEFAULT_TOPIC_CAM_0)
        self.declare_parameter('cam_0.model_path', DEFAULT_MODL_PATH)
        self.cam_0_topic = self.get_parameter('cam_0.topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('cam_0.model_path').get_parameter_value().string_value

        self.get_logger().info(f"CAM_0_TOPIC: {self.cam_0_topic}, MODEL_PATH: {self.model_path}")

        self.subscription = self.create_subscription(Image,self.cam_0_topic, self.listener_callback, 1)
        self.subscription  
        self.bridge = CvBridge()
        self.current_frame = None
        self.results = None
        self.processed_frame = None
    
        self.yoloInference = YoloInference(self.model_path)
      

    def listener_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info('Received frame from cam_0')

        self.results, self.processed_frame = self.yoloInference.get_processed_frame(frame=self.current_frame)
        
        while self.results is None or self.processed_frame is None:
            self.get_logger().info('Waiting for results from YOLO')
            self.get_logger().info('Waiting for frame from cam_0')
            continue
            
        
        cv2.imshow('Subscribed Frame cam_0', self.current_frame)
        cv2.imshow('Result', self.processed_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    frame_subscriber = FrameSubscriber()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(frame_subscriber)
    try:
        executor.spin()

    finally:
        frame_subscriber.destroy_node()
        rclpy.shutdown()

 

if __name__ == '__main__':
    main()