#!/usr/bin/env python3
"""
Motor Controller Node for ROS.

Converts high-level navigation commands to low-level motor commands.
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
import logging

logger = logging.getLogger(__name__)


class ControllerNode:
    """
    ROS node for motor control.
    
    Subscribes to:
        - /planned_path: Planned path for navigation
        - /slam/pose: Current robot pose
        
    Publishes to:
        - /cmd_vel: Robot velocity commands
    """

    def __init__(self, node_name: str = "controller_node"):
        """Initialize the controller node."""
        rospy.init_node(node_name, anonymous=True)
        
        # Parameters
        self.max_linear_vel = rospy.get_param("~max_linear_vel", 0.5)
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 1.0)
        self.control_rate = rospy.get_param("~control_rate", 10.0)
        
        # State
        self.current_pose = None
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10
        )
        
        # Subscribers
        self.pose_sub = rospy.Subscriber(
            "/slam/pose",
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )
        
        # Timer for control loop
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_callback
        )
        
        rospy.loginfo("Controller node initialized successfully")

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for pose updates.
        
        Args:
            msg: PoseStamped message
        """
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def control_callback(self, timer_event):
        """
        Main control loop callback.
        
        Args:
            timer_event: Timer event
        """
        if self.current_pose is None:
            rospy.logdebug("Waiting for pose")
            return

        # Create velocity command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

    def run(self):
        """Run the node."""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ControllerNode()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Controller node shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")