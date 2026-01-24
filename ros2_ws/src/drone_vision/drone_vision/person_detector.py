#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # Load YOLOv8 model (pretrained on COCO)
        # 'yolov8n.pt' is the nano version (fastest, less accurate). 
        # Will download automatically on first run.
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/image_debug', 10)

        self.get_logger().info("Person Detector Node Started (YOLOv8n)")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run Inference
            # classes=0 limits detection to 'person' only (COCO class 0)
            results = self.model(cv_image, classes=0, verbose=False)
            
            # Construct Detection2DArray
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            result = results[0] # We only processing one image
            
            for box in result.boxes:
                # YOLOv8 returns [x1, y1, x2, y2]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())

                # Calculate center and size for BoundingBox2D
                w = x2 - x1
                h = y2 - y1
                cx = x1 + (w / 2)
                cy = y1 + (h / 2)

                detection = Detection2D()
                detection.header = msg.header
                
                # Bounding Box
                detection.bbox = BoundingBox2D()
                detection.bbox.center.position.x = float(cx)
                detection.bbox.center.position.y = float(cy)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                # Hypothesis
                mph = ObjectHypothesisWithPose()
                mph.hypothesis.class_id = self.model.names[cls]
                mph.hypothesis.score = conf
                detection.results.append(mph)

                detections_msg.detections.append(detection)

                # Draw debug box
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{conf:.2f}", (int(x1), int(y1)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish detections
            self.detection_pub.publish(detections_msg)
            
            # Publish debug image
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
