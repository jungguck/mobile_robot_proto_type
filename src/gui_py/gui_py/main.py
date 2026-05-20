import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .teleop_control import TeleopController
from .mpc_control import MPCController
from .slam_control import ProcessLauncher


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_py_node')

        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        self.teleop = TeleopController(self)
        self.slam_launcher = ProcessLauncher(
            ['ros2', 'launch', 'relayrobot_description', 'cartographer.launch.py'],
            'SLAM'
        )
        self.lidar_launcher = ProcessLauncher(
            ['ros2', 'run', 'sllidar_ros2', 'sllidar_node'],
            'LIDAR'
        )
        self.imu_launcher = ProcessLauncher(
            ['ros2', 'run', 'ebimu_pkg', 'ebimu_publisher'],
            'IMU'
        )
        self.mpc_controller = MPCController(self)

        # Odom subscription
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10
        )

        self.root = tk.Tk()
        self.root.title('Relay Robot GUI')
        self.root.geometry('480x450')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._build_widgets()
        
        # Start a thread to spin rclpy
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def _spin(self):
        rclpy.spin(self)

    def _odom_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
        
        # Update MPC controller's knowledge of the pose
        self.mpc_controller.update_pose(self.current_pose)
        
        # Update UI (thread-safe way)
        try:
            self.root.after(0, self._update_pose_ui)
        except:
            pass

    def _update_pose_ui(self):
        pose_text = f"X: {self.current_pose['x']:.2f}, Y: {self.current_pose['y']:.2f}, Yaw: {math.degrees(self.current_pose['theta']):.1f}°"
        self.pose_label.config(text=pose_text)

    def _build_widgets(self):
        # Pose Info Frame
        frame_pose = tk.LabelFrame(self.root, text='Robot Status (Odom)', padx=8, pady=8)
        frame_pose.pack(fill='x', padx=10, pady=6)
        self.pose_label = tk.Label(frame_pose, text="X: 0.00, Y: 0.00, Yaw: 0.0°", font=('Courier', 10, 'bold'))
        self.pose_label.pack()

        frame_drive = tk.LabelFrame(self.root, text='Teleop Control', padx=8, pady=8)
        frame_drive.pack(fill='x', padx=10, pady=6)

        buttons = [
            ('Forward', lambda: self.teleop.send(linear=0.2)),
            ('Backward', lambda: self.teleop.send(linear=-0.2)),
            ('Left', lambda: self.teleop.send(angular=0.4)),
            ('Right', lambda: self.teleop.send(angular=-0.4)),
            ('Stop', lambda: self.teleop.send(linear=0.0, angular=0.0)),
        ]

        for text, command in buttons:
            btn = tk.Button(frame_drive, text=text, width=10, command=command)
            btn.pack(side='left', padx=4, pady=4)

        frame_slam = tk.LabelFrame(self.root, text='SLAM / Sensor Control', padx=8, pady=8)
        frame_slam.pack(fill='x', padx=10, pady=6)

        tk.Button(frame_slam, text='Start SLAM', width=12, command=self._start_slam).pack(side='left', padx=4, pady=4)
        tk.Button(frame_slam, text='Stop SLAM', width=12, command=self._stop_slam).pack(side='left', padx=4, pady=4)
        tk.Button(frame_slam, text='Start LIDAR', width=12, command=self._start_lidar).pack(side='left', padx=4, pady=4)
        tk.Button(frame_slam, text='Start IMU', width=12, command=self._start_imu).pack(side='left', padx=4, pady=4)

        frame_mpc = tk.LabelFrame(self.root, text='MPC / Demo', padx=8, pady=8)
        frame_mpc.pack(fill='x', padx=10, pady=6)

        tk.Button(frame_mpc, text='Start MPC', width=12, command=self._start_mpc).pack(side='left', padx=4, pady=4)
        tk.Button(frame_mpc, text='Stop MPC', width=12, command=self._stop_mpc).pack(side='left', padx=4, pady=4)

        self.status_label = tk.Label(self.root, text='Status: ready', anchor='w')
        self.status_label.pack(fill='x', padx=10, pady=8)

    def _start_slam(self):
        success, msg = self.slam_launcher.start()
        self._set_status(msg)

    def _stop_slam(self):
        success, msg = self.slam_launcher.stop()
        self._set_status(msg)

    def _start_lidar(self):
        success, msg = self.lidar_launcher.start()
        self._set_status(msg)

    def _start_imu(self):
        success, msg = self.imu_launcher.start()
        self._set_status(msg)

    def _start_mpc(self):
        if self.mpc_controller.start():
            self._set_status('MPC demo started')
        else:
            self._set_status('MPC already running or failed')

    def _stop_mpc(self):
        if self.mpc_controller.stop():
            self._set_status('MPC demo stopped')
        else:
            self._set_status('MPC not running')

    def _set_status(self, text):
        self.status_label.config(text=f'Status: {text}')

    def _on_close(self):
        self.mpc_controller.stop()
        self.slam_launcher.stop()
        self.lidar_launcher.stop()
        self.imu_launcher.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    try:
        node.run()
    finally:
        # Node destruction handled by rclpy.shutdown if spin thread is daemon
        pass


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
