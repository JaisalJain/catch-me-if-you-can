"""catch_ballistics: projectile model and release/intercept solvers (numpy/scipy only, no ROS)."""
from .projectile import G, Projectile
from .solver import (InterceptCandidate, choose_intercept, intercepts_in_box, min_speed_angle,
                     velocity_for_flight_time, velocity_for_launch_angle)

__all__ = ["G", "Projectile", "InterceptCandidate", "choose_intercept", "intercepts_in_box", "min_speed_angle",
           "velocity_for_flight_time", "velocity_for_launch_angle"]
