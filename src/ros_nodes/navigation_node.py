#!/usr/bin/env python3
"""
Navigation Node for ROS.

Handles path planning and publishes navigation commands.
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_srvs.srv import Empty, EmptyResponse
import logging
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class NavigationNode:
    """
    ROS node for navigation and path planning.
    
    Subscribes to:
        - /map: Occupancy grid map
        - /slam/pose: Current robot pose
        
    Publishes to:
        - /planned_path: Planned path as nav_msgs/Path
    """

    def __init__(self, node_name: str = "navigation_node"):
        """Initialize the navigation node."""
        rospy.init_node(node_name, anonymous=True)
        
        # Parameters
        self.planning_algorithm = rospy.get_param("~algorithm", "a_star")
        
        # Initialize path planner
        try:
            from src.navigation import PathPlanner, PlanningAlgorithm
            self.path_planner = PathPlanner(
                grid_size=(100, 100),
                cell_size=0.1,
                algorithm=PlanningAlgorithm[self.planning_algorithm.upper()]
            )
        except Exception as e:
            rospy.logerr(f"Failed to initialize path planner: {e}")
            raise
        
        # State
        self.current_pose: Optional[Tuple[float, float]] = None
        self.occupancy_grid: Optional[np.ndarray] = None
        
        # Publishers
        self.path_pub = rospy.Publisher(
            "/planned_path",
            Path,
            queue_size=10
        )
        
        # Subscribers
        self.map_sub = rospy.Subscriber(
            "/map",
            OccupancyGrid,
            self.map_callback,
            queue_size=1
        )
        
        self.pose_sub = rospy.Subscriber(
            "/slam/pose",
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )
        
        rospy.loginfo("Navigation node initialized successfully")

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback for occupancy grid updates.
        
        Args:
            msg: OccupancyGrid message
        """
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.uint8).reshape((height, width))
        grid = (grid > 50).astype(np.uint8)
        self.occupancy_grid = grid
        rospy.logdebug(f"Updated occupancy grid: {grid.shape}")

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for pose updates.
        
        Args:
            msg: PoseStamped message
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.current_pose = (x, y)
        rospy.logdebug(f"Robot pose updated: {x:.2f}, {y:.2f}")

    def _publish_path(self, path: list):
        """
        Publish planned path as ROS Path message.
        
        Args:
            path: List of (x, y) waypoints
        """
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def run(self):
        """Run the node."""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = NavigationNode()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Navigation node shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")