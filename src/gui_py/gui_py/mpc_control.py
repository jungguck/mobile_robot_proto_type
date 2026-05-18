import threading
import time

from geometry_msgs.msg import Twist


class MPCController:
    def __init__(self, node):
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.thread = None
        self.stop_event = threading.Event()

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
        return True

    def _run_loop(self):
        rate_hz = 10.0
        while not self.stop_event.is_set():
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            time.sleep(1.0 / rate_hz)
