# 🎯 Drone Vision Package

ROS 2 package for real-time person detection and tracking using YOLOv8 and GPU acceleration.

## Features

- ✅ Real-time person detection using YOLOv8
- ✅ GPU-accelerated inference (CUDA 12.1)
- ✅ ROS 2 Jazzy compatible
- ✅ Publishes detection results and annotated images
- ✅ Configurable confidence threshold and model
- ✅ Center-point tracking for drone control

## Installation

Already installed if you ran the setup script! Otherwise:

```bash
cd ~/Drone/ros2_ws
colcon build --packages-select drone_vision
source install/setup.bash
```

## Usage

### 1. Basic Usage (with camera topic)

```bash
# Terminal 1: Run person detector
ros2 run drone_vision person_detector

# Terminal 2: Publish test camera feed (if you have a camera)
ros2 run usb_cam usb_cam_node_exe
```

### 2. Test with Video File

```bash
# Create a test video publisher (install if needed: sudo apt install ros-jazzy-image-tools)
ros2 run image_tools cam2image --ros-args -p frequency:=30.0

# Or publish from a video file
ros2 run image_publisher image_publisher_node --ros-args -p filename:=/path/to/video.mp4
```

### 3. View Detections

```bash
# View detection messages
ros2 topic echo /detections

# View annotated image (requires RViz2 or rqt)
ros2 run rqt_image_view rqt_image_view /detections/image
```

### 4. With Parameters

```bash
ros2 run drone_vision person_detector --ros-args \
  -p model:=yolov8n.pt \
  -p confidence:=0.5 \
  -p device:=cuda:0 \
  -p show_preview:=true
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | `yolov8n.pt` | YOLO model (n=nano, s=small, m=medium, l=large) |
| `confidence` | float | `0.5` | Minimum detection confidence (0.0-1.0) |
| `device` | string | `cuda:0` | Device for inference (cuda:0 or cpu) |
| `show_preview` | bool | `false` | Show OpenCV preview window |

## Topics

### Subscribed
- `/camera/image_raw` (sensor_msgs/Image) - Input camera stream

### Published
- `/detections` (std_msgs/String) - Detection summary text
- `/detections/image` (sensor_msgs/Image) - Annotated image with bounding boxes

## Testing

### Test with Gazebo Camera

1. Launch Gazebo with a camera-equipped drone:
```bash
gz sim camera_sensor.sdf
```

2. Run the detector:
```bash
ros2 run drone_vision person_detector
```

### Test with Static Image

Create a test publisher:

```python
# test_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.bridge = CvBridge()
        
        # Load test image (replace with your image path)
        self.image = cv2.imread('test_image.jpg')
        
    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Performance

On RTX 2060:
- **YOLOv8n**: ~80-100 FPS
- **YOLOv8s**: ~60-70 FPS
- **YOLOv8m**: ~40-50 FPS

## Troubleshooting

### No detections appearing

1. Check camera topic is publishing:
```bash
ros2 topic list
ros2 topic hz /camera/image_raw
```

2. Check if images are being received:
```bash
ros2 topic echo /camera/image_raw --no-arr
```

3. Lower confidence threshold:
```bash
ros2 run drone_vision person_detector --ros-args -p confidence:=0.3
```

### CUDA errors

Verify GPU is accessible:
```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

If False, check:
```bash
nvidia-smi
nvcc --version
```

### Slow performance

1. Use smaller model:
```bash
ros2 run drone_vision person_detector --ros-args -p model:=yolov8n.pt
```

2. Verify GPU is being used:
```bash
nvidia-smi  # Check GPU utilization while running
```

## Next Steps

- [ ] Add Kalman filter for tracking smoothing
- [ ] Implement multi-person tracking with IDs
- [ ] Add distance estimation using depth camera
- [ ] Create action server for "follow person" behavior
- [ ] Integrate with PX4 for autonomous tracking

## License

Apache 2.0
