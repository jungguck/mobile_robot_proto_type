import threading
import time
import math
import numpy as np
from geometry_msgs.msg import Twist

class MPCController:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.thread = None
        self.stop_event = threading.Event()
        self.current_pose = np.array([0.0, 0.0, 0.0]) # x, y, theta
        
        # Reference trajectory (simplified: a straight line for demo)
        self.target_pose = np.array([2.0, 0.0, 0.0]) # Target 2m ahead
        
    def update_pose(self, pose_dict):
        self.current_pose = np.array([
            pose_dict['x'],
            pose_dict['y'],
            pose_dict['theta']
        ])

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return False

        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        if self.thread is None or not self.thread.is_alive():
            return False

        self.stop_event.set()
        self.thread.join(timeout=2.0)
        
        # Stop the robot
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        return True

    def _run_loop(self):
        rate_hz = 10.0
        dt = 1.0 / rate_hz
        
        self.node.get_logger().info("MPC Loop Started")
        
        while not self.stop_event.is_set():
            # 1. Calculate Error in Robot Frame
            dx = self.target_pose[0] - self.current_pose[0]
            dy = self.target_pose[1] - self.current_pose[1]
            
            # Distance to target
            dist = math.sqrt(dx**2 + dy**2)
            
            # Angle to target
            angle_to_target = math.atan2(dy, dx)
            angle_error = self._wrap_angle(angle_to_target - self.current_pose[2])
            
            # 2. Simple Control Law (Proportional)
            # This serves as a robust fallback if MPC solver is not available
            twist = Twist()
            
            if dist > 0.1:
                # If angle error is large, turn first
                if abs(angle_error) > 0.5:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.4 * np.sign(angle_error)
                else:
                    twist.linear.x = min(0.2, 0.5 * dist)
                    twist.angular.z = 0.5 * angle_error
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.node.get_logger().info("Target Reached")
                # We could auto-stop here or wait for next target
            
            self.publisher.publish(twist)
            time.sleep(dt)

    def _wrap_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
