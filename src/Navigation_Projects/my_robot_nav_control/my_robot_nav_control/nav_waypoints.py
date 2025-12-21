#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


class NavigationTask(Node):

    def __init__(self):
        super().__init__('nav_waypoints_node')

        self.navigator = BasicNavigator()

        # Declare parameters as lists (not strings)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('waypoints', [])
        self.declare_parameter('final_pose', [0.0, 0.0, 0.0])

        # Read parameters (already parsed)
        initial_pose_val = self.get_parameter('initial_pose').value
        waypoints_val = self.get_parameter('waypoints').value
        final_pose_val = self.get_parameter('final_pose').value

        self.initial_pose = self._parse_points(initial_pose_val, expect='pose')
        self.waypoints = self._parse_points(waypoints_val, expect='waypoints')
        self.final_pose = self._parse_points(final_pose_val, expect='pose')

        self.get_logger().info(f"Initial pose: {self.initial_pose}")
        self.get_logger().info(f"Waypoints: {self.waypoints}")
        self.get_logger().info(f"Final pose: {self.final_pose}")

    def _parse_points(self, val, expect: str):
        """
        expect='pose'      -> expects [x,y,yaw] and returns (x,y,yaw)
        expect='waypoints' -> expects [] or [[x,y,yaw], ...] and returns [(x,y,yaw), ...]
        """
        if expect == 'pose':
            if not isinstance(val, (list, tuple)) or len(val) != 3:
                raise ValueError(f'Expected [x,y,yaw]. Got: {val}')
            return (float(val[0]), float(val[1]), float(val[2]))

        if expect == 'waypoints':
            if val is None:
                return []
            if not isinstance(val, list):
                raise ValueError(f'Expected list of waypoints. Got: {val}')
            parsed = []
            for item in val:
                if not isinstance(item, (list, tuple)) or len(item) != 3:
                    raise ValueError(f'Each waypoint must be [x,y,yaw]. Got: {item}')
                parsed.append((float(item[0]), float(item[1]), float(item[2])))
            return parsed

        raise ValueError(f'Unknown expect: {expect}')

    def _create_pose_stamped(self, x, y, yaw):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        return pose

    def set_initial_pose(self):
        x, y, yaw = self.initial_pose
        self.navigator.setInitialPose(self._create_pose_stamped(x, y, yaw))
        self.get_logger().info(f"Initial pose set: {self.initial_pose}")

    def wait_for_nav2(self):
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

    def run_navigation(self):
        final_x, final_y, final_yaw = self.final_pose

        if len(self.waypoints) == 0:
            self.get_logger().info("No waypoints. Going directly to final pose.")
            goal = self._create_pose_stamped(final_x, final_y, final_yaw)
            self.navigator.goToPose(goal)
        else:
            self.get_logger().info(f"Following {len(self.waypoints)} waypoints and then final pose.")
            pose_list = [self._create_pose_stamped(x, y, yaw) for (x, y, yaw) in self.waypoints]
            pose_list.append(self._create_pose_stamped(final_x, final_y, final_yaw))
            self.navigator.followWaypoints(pose_list)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation finished: {result}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTask()

    node.set_initial_pose()
    node.wait_for_nav2()
    node.run_navigation()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
