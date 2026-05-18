from geometry_msgs.msg import Twist


class TeleopController:
    def __init__(self, node):
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
