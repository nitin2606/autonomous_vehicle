import rclpy
import rclpy.executors
from rclpy.node import Node
from .camera import Camera



class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.cameras: list[Camera] = []

        self.declare_parameter('camera_count', 1)
        camera_count = self.get_parameter('camera_count').get_parameter_value().integer_value

        for i in range(camera_count):  
            self.camera_id = f'cam_{i}'
            self.declare_parameter(f'{self.camera_id}.camera_index', 0)
            self.declare_parameter(f'{self.camera_id}.frame_rate', 20)
            self.declare_parameter(f'{self.camera_id}.video_file', '')
            self.declare_parameter(f'{self.camera_id}.video_url', '')

        
        for i in range(camera_count):
            self.camera_id = f'cam_{i}'
            self.camera_index = self.get_parameter(f'{self.camera_id}.camera_index').get_parameter_value().integer_value
            self.frame_rate = self.get_parameter(f'{self.camera_id}.frame_rate').get_parameter_value().integer_value
            self.video_file = self.get_parameter(f'{self.camera_id}.video_file').get_parameter_value().string_value
            self.video_url = self.get_parameter(f'{self.camera_id}.video_url').get_parameter_value().string_value

            self.get_logger().info(f"Camera_Id: {self.camera_id}, Camera_Index: {self.camera_index}, Frame_Rate: {self.frame_rate}, Video_File: {self.video_file}, Video_URL: {self.video_url}")

            camera = Camera(self, self.camera_id, self.camera_index, self.video_file, self.video_url)
            self.cameras.append(camera)

        self.timer = self.create_timer(1/self.frame_rate, self.timer_callback)

    def timer_callback(self):
        for camera in self.cameras:
            camera.read_and_publish()

    def __del__(self):
        for camera in self.cameras:
            camera.release()

def main(args=None):
    rclpy.init(args=args)
    frame_publisher = FramePublisher()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(frame_publisher)
    try:
        executor.spin()
    finally:
        frame_publisher.destroy_node()
        rclpy.shutdown()
  

if __name__ == '__main__':
    main()