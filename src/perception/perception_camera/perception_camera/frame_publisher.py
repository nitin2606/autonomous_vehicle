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
            camera_id = f'cam_{i}'
            self.declare_parameter(f'{camera_id}.camera_index', 0)
            self.declare_parameter(f'{camera_id}.video_file', '')
            self.declare_parameter(f'{camera_id}.video_url', '')

        
        for i in range(camera_count):
            camera_id = f'cam_{i}'
            camera_index = self.get_parameter(f'{camera_id}.camera_index').get_parameter_value().integer_value
            video_file = self.get_parameter(f'{camera_id}.video_file').get_parameter_value().string_value
            video_url = self.get_parameter(f'{camera_id}.video_url').get_parameter_value().string_value

            self.get_logger().info(f"Camera_Id: {camera_id}, Camera_Index: {camera_index}, Video_File: {video_file}, Video_URL: {video_url}")

            camera = Camera(self, camera_id, camera_index, video_file, video_url)
            self.cameras.append(camera)

        self.timer = self.create_timer(1/30, self.timer_callback)

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
    # rclpy.spin(frame_publisher)
    # frame_publisher.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()