import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera:
    def __init__(self, node, camera_id, camera_index, video_file, video_url):
        self.node = node
        self.camera_id = camera_id
        self.publisher = node.create_publisher(Image, f'{camera_id}/frame', 1)
        self.bridge = CvBridge()

        if video_file:
            self.cap = cv2.VideoCapture(video_file)
            
        elif video_url:
            self.cap = cv2.VideoCapture(video_url)

        else:
            self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            self.node.get_logger().error(f'Failed to open camera: {camera_id}')
    
   
    def read_and_publish(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.node.get_logger().info(f"Publishing Camera Frame From Camera: {self.camera_id} on Topic: {self.camera_id}/frame")
        else:
            self.node.get_logger().error(f'Failed to read frame from camera: {self.camera_id}')

    def release(self):
        self.cap.release()