#!/usr/bin/env python3
"""
Vision Node for ROS.

Handles real-time camera input, object detection, and publishes
detection results for navigation pipeline.
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class VisionNode:
    """
    ROS node for vision pipeline.
    
    Subscribes to:
        - /camera/image_raw: Raw camera frames
        
    Publishes to:
        - /detector/objects: Detected objects
        - /detector/image: Debug visualization
    """

    def __init__(self, node_name: str = "vision_node"):
        """Initialize the vision node."""
        rospy.init_node(node_name, anonymous=True)
        
        # Parameters
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.detector_model = rospy.get_param("~model_path", "yolov8m.pt")
        self.conf_threshold = rospy.get_param("~conf_threshold", 0.5)
        self.publish_viz = rospy.get_param("~publish_visualization", True)
        
        # Initialize bridge and detector
        self.bridge = CvBridge()
        
        try:
            from src.vision import ObjectDetector
            self.detector = ObjectDetector(
                model_path=self.detector_model,
                conf_threshold=self.conf_threshold,
            )
        except Exception as e:
            rospy.logerr(f"Failed to initialize detector: {e}")
            raise
        
        # Publishers
        self.detection_pub = rospy.Publisher(
            "/detector/objects",
            Image,
            queue_size=10
        )
        self.viz_pub = rospy.Publisher(
            "/detector/image",
            Image,
            queue_size=10
        )
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            self.camera_topic,
            Image,
            self.image_callback,
            queue_size=1
        )
        
        rospy.loginfo("Vision node initialized successfully")

    def image_callback(self, msg: Image):
        """
        Callback for camera images.
        
        Args:
            msg: ROS Image message
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # Run detection
        detections = self.detector.detect(cv_image)

        # Publish visualization if enabled
        if self.publish_viz:
            viz_image = self.detector.detect_with_visualization(
                cv_image,
                detections
            )
            try:
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, "bgr8")
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to publish visualization: {e}")

    def run(self):
        """Run the node."""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = VisionNode()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Vision node shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")