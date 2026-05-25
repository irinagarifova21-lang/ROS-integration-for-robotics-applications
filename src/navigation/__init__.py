"""
Navigation module for AI-based path planning and obstacle avoidance.
"""

from .path_planner import PathPlanner, PlanningAlgorithm

__all__ = [
    'PathPlanner',
    'PlanningAlgorithm',
]

__version__ = '1.0.0'