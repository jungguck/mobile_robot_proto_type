import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node

from .teleop_control import TeleopController
from .mpc_control import MPCController
from .slam_control import ProcessLauncher


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_py_node')

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

        self.root = tk.Tk()
        self.root.title('Relay Robot GUI')
        self.root.geometry('480x380')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._build_widgets()

    def _build_widgets(self):
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
        if self.slam_launcher.start():
            self._set_status('SLAM started')
        else:
            self._set_status('SLAM already running')

    def _stop_slam(self):
        if self.slam_launcher.stop():
            self._set_status('SLAM stopped')
        else:
            self._set_status('SLAM not running')

    def _start_lidar(self):
        if self.lidar_launcher.start():
            self._set_status('LIDAR started')
        else:
            self._set_status('LIDAR already running')

    def _start_imu(self):
        if self.imu_launcher.start():
            self._set_status('IMU started')
        else:
            self._set_status('IMU already running')

    def _start_mpc(self):
        if self.mpc_controller.start():
            self._set_status('MPC demo started')
        else:
            self._set_status('MPC already running')

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
