import heapq
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('mpc_tubempc_path_planner')

        self.map = None
        self.map_info = None
        self.goal = None
        self.pose = None

        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(PoseStamped, 'mpc_goal', self.goal_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, 'global_path', 10)

        self.timer = self.create_timer(0.5, self.plan_callback)
        self.get_logger().info('A* Path Planner initialized.')

    def map_callback(self, msg: OccupancyGrid):
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.get_logger().info('Map received.')

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg.pose
        self.get_logger().info(f'Goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    def plan_callback(self):
        if self.map is None or self.goal is None or self.pose is None:
            return

        start = self.world_to_map(self.pose.position.x, self.pose.position.y)
        goal = self.world_to_map(self.goal.position.x, self.goal.position.y)

        if not self.is_free(start) or not self.is_free(goal):
            self.get_logger().warn('Start or goal cell is not free.')
            return

        path = self.a_star(start, goal)
        if path is None:
            self.get_logger().warn('Path planning failed.')
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for mx, my in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y = self.map_to_world(mx, my)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published global path ({len(path)} points).')

    def world_to_map(self, x: float, y: float):
        origin = self.map_info.origin.position
        res = self.map_info.resolution
        mx = int((x - origin.x) / res)
        my = int((y - origin.y) / res)
        return mx, my

    def map_to_world(self, mx: int, my: int):
        origin = self.map_info.origin.position
        res = self.map_info.resolution
        x = origin.x + (mx + 0.5) * res
        y = origin.y + (my + 0.5) * res
        return x, y

    def is_free(self, cell):
        x, y = cell
        if x < 0 or y < 0 or x >= self.map_info.width or y >= self.map_info.height:
            return False
        value = self.map[y, x]
        return value == -1 or value < 50

    def neighbors(self, cell):
        x, y = cell
        for dx, dy, cost in [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height and self.is_free((nx, ny)):
                yield (nx, ny), cost

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def a_star(self, start, goal):
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor, cost in self.neighbors(current):
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
