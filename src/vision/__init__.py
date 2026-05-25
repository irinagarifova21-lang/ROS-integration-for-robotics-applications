"""
Vision module for AI Navigation with OpenCV.
Implements object detection, visual odometry, and semantic segmentation.
"""

from .object_detector import ObjectDetector

__all__ = [
    'ObjectDetector',
]

__version__ = '1.0.0'