#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        # Subscribe to RGB image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgbd/image_raw',
            self.image_callback,
            10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections_2d',
            10
        )
        
        # CV Bridge for ROS<->OpenCV conversion
        self.bridge = CvBridge()
        
        # HSV color ranges for detection
        # [H_min, S_min, V_min], [H_max, S_max, V_max]
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 100, 100]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([160, 100, 100]),
                'upper2': np.array([180, 255, 255])
            },
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            'blue': {
                'lower': np.array([90, 50, 50]),
                'upper': np.array([130, 255, 255])
            }
        }
        
        self.get_logger().info('Color detector node started')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Detect objects
        detections = Detection2DArray()
        detections.header = msg.header
        
        # Detect red (needs two ranges due to hue wraparound)
        red_mask1 = cv2.inRange(hsv, self.color_ranges['red']['lower1'], 
                                self.color_ranges['red']['upper1'])
        red_mask2 = cv2.inRange(hsv, self.color_ranges['red']['lower2'], 
                                self.color_ranges['red']['upper2'])
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        self.detect_object(red_mask, 'red_cube', detections, msg.header)
        
        # Detect green
        green_mask = cv2.inRange(hsv, self.color_ranges['green']['lower'], 
                                 self.color_ranges['green']['upper'])
        self.detect_object(green_mask, 'green_sphere', detections, msg.header)
        
        # Detect blue
        blue_mask = cv2.inRange(hsv, self.color_ranges['blue']['lower'], 
                               self.color_ranges['blue']['upper'])
        self.detect_object(blue_mask, 'blue_cylinder', detections, msg.header)
        
        # Publish detections
        self.detection_pub.publish(detections)
    
    def detect_object(self, mask, color_name, detections, header):
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return
        
        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter small detections (noise)
        if area < 100:
            return
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Create detection message
        detection = Detection2D()
        detection.header = header
        
        # Bounding box center
        detection.bbox.center.position.x = float(x + w / 2)
        detection.bbox.center.position.y = float(y + h / 2)
        detection.bbox.size_x = float(w)
        detection.bbox.size_y = float(h)
        
        # Object hypothesis
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = color_name
        hypothesis.hypothesis.score = 1.0  # We're confident in color detection
        detection.results.append(hypothesis)
        
        detections.detections.append(detection)
        
        self.get_logger().debug(f'Detected {color_name} at ({x+w/2:.1f}, {y+h/2:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()