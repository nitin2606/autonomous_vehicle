
# Monocular VSLAM Package

This package implements monocular visual SLAM using ROS2 Humble and ORB-SLAM3.

## Setup

1. Clone this repository:
   ```bash
   git clone <repository-url> ~/ros2_ws/src
   ```
2. Download `ORBvoc.txt`:
   ```bash
   cd ~/ros2_ws/src/monocular_vslam/config
   wget https://github.com/raulmur/ORB_SLAM2/releases/download/1.0/ORBvoc.txt.tar.gz
   tar -xvzf ORBvoc.txt.tar.gz
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. Launch the SLAM system:
   ```bash
   ros2 launch monocular_vslam vslam_launch.py
   ```

## Customization

- Update `camera_calibration.yaml` to use your camera's intrinsic parameters.
- Modify `vslam_launch.py` if using a different camera node.