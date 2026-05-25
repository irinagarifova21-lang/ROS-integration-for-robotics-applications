#!/usr/bin/env python3
"""
Object Detection Module using YOLOv8.

This module provides real-time object detection capabilities for obstacle
and landmark detection in the robot's environment.
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from ultralytics import YOLO
import logging

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """Data class for a single detection result."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # (x1, y1, x2, y2)
    center: Tuple[float, float]
    area: float

    def __post_init__(self):
        """Calculate area from bounding box."""
        x1, y1, x2, y2 = self.bbox
        self.area = (x2 - x1) * (y2 - y1)
        self.center = ((x1 + x2) / 2, (y1 + y2) / 2)


class ObjectDetector:
    """
    Real-time object detector using YOLOv8.
    
    Supports detection of obstacles, landmarks, and other relevant objects
    for autonomous navigation.
    
    Attributes:
        model: YOLOv8 model instance
        conf_threshold: Confidence threshold for detections
        device: Device for inference (cpu, cuda, etc.)
    """

    def __init__(
        self,
        model_path: str = "yolov8m.pt",
        conf_threshold: float = 0.5,
        device: str = "cuda",
        max_detections: int = 100,
    ):
        """
        Initialize the object detector.

        Args:
            model_path: Path to YOLOv8 model weights
            conf_threshold: Confidence threshold for filtering detections
            device: Device for inference
            max_detections: Maximum number of detections to return
        """
        try:
            self.model = YOLO(model_path)
            self.model.to(device)
            self.conf_threshold = conf_threshold
            self.device = device
            self.max_detections = max_detections
            logger.info(f"ObjectDetector initialized with {model_path} on {device}")
        except Exception as e:
            logger.error(f"Failed to initialize ObjectDetector: {e}")
            raise

    def detect(
        self,
        image: np.ndarray,
        classes: Optional[List[int]] = None,
        track: bool = False,
    ) -> List[Detection]:
        """
        Detect objects in the input image.

        Args:
            image: Input image (numpy array, BGR format)
            classes: List of class IDs to detect (None for all)
            track: Whether to use tracking (persistent IDs)

        Returns:
            List of Detection objects
        """
        try:
            # Run inference
            results = self.model(image, conf=self.conf_threshold, verbose=False)

            detections = []
            if results[0].boxes is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                confs = results[0].boxes.conf.cpu().numpy()
                class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
                class_names = results[0].names

                # Filter by class if specified
                for box, conf, class_id in zip(boxes, confs, class_ids):
                    if classes is not None and class_id not in classes:
                        continue

                    if len(detections) >= self.max_detections:
                        break

                    detection = Detection(
                        class_id=class_id,
                        class_name=class_names[class_id],
                        confidence=float(conf),
                        bbox=tuple(box),
                    )
                    detections.append(detection)

            return sorted(detections, key=lambda x: x.confidence, reverse=True)

        except Exception as e:
            logger.error(f"Error during detection: {e}")
            return []

    def detect_with_visualization(
        self,
        image: np.ndarray,
        detections: Optional[List[Detection]] = None,
        draw_confidence: bool = True,
    ) -> np.ndarray:
        """
        Draw detections on the image.

        Args:
            image: Input image
            detections: List of detections (if None, will run detection)
            draw_confidence: Whether to draw confidence scores

        Returns:
            Image with drawn detections
        """
        if detections is None:
            detections = self.detect(image)

        img_copy = image.copy()

        for det in detections:
            x1, y1, x2, y2 = det.bbox
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Draw bounding box
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            label = f"{det.class_name}"
            if draw_confidence:
                label += f" {det.confidence:.2f}"

            cv2.putText(
                img_copy,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            # Draw center point
            cx, cy = det.center
            cv2.circle(img_copy, (int(cx), int(cy)), 5, (255, 0, 0), -1)

        return img_copy

    def filter_detections(
        self,
        detections: List[Detection],
        class_names: Optional[List[str]] = None,
        min_area: float = 0,
        max_area: float = float("inf"),
    ) -> List[Detection]:
        """
        Filter detections by various criteria.

        Args:
            detections: List of detections to filter
            class_names: List of class names to keep
            min_area: Minimum bounding box area
            max_area: Maximum bounding box area

        Returns:
            Filtered list of detections
        """
        filtered = detections

        if class_names:
            filtered = [d for d in filtered if d.class_name in class_names]

        filtered = [
            d for d in filtered if min_area <= d.area <= max_area
        ]

        return filtered

    def update_confidence_threshold(self, new_threshold: float):
        """Update the confidence threshold."""
        self.conf_threshold = max(0.0, min(1.0, new_threshold))
        logger.info(f"Confidence threshold updated to {self.conf_threshold}")

    def get_model_info(self) -> Dict:
        """Get information about the loaded model."""
        return {
            "model_type": self.model.model_name,
            "task": self.model.task,
            "device": self.device,
            "conf_threshold": self.conf_threshold,
            "classes": self.model.names,
        }