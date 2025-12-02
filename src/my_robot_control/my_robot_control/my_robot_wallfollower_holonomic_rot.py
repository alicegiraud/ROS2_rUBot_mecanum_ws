import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


class Rotate90Node(Node):
    def __init__(self):
        super().__init__('rotate_90_node')

        # Parameters
        self.declare_parameter('forward_speed', 0.10)
        self.declare_parameter('strafe_speed', 0.10)
        self.declare_parameter('turn_speed', 0.40)   # rad/s
        self.declare_parameter('move_duration', 3.0) # seconds in same direction

        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_strafe = float(self.get_parameter('strafe_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.move_duration = float(self.get_parameter('move_duration').value)

        # Time needed to rotate 90 deg at v_ang
        # 90° = pi/2 rad → t = (pi/2)/v_ang
        self.rotation_time_90 = (math.pi / 2.0) / self.v_ang

        # Publisher
        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        # State
        self.cmd = Twist()
        self.current_dir = "forward"   # "forward", "backward", "left", "right"
        self.dir_start_time = self.get_clock().now().nanoseconds * 1e-9

        self.rotating = False
        self.rotation_dir = 1.0        # +1 = left, -1 = right
        self.rotation_start_time = 0.0

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Rotate90Node started.")

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        twist = Twist()

        if self.rotating:
            # Rotation phase: rotate with constant angular speed for rotation_time_90
            elapsed_rot = now - self.rotation_start_time
            if elapsed_rot >= self.rotation_time_90:
                # Stop rotation
                self.rotating = False
                twist.angular.z = 0.0

                # After rotating, choose a new movement direction if you want
                # For example, keep same linear direction but rotated logically,
                # or just stop:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.get_logger().info("Finished 90° rotation.")
            else:
                # Keep rotating
                twist.angular.z = self.rotation_dir * self.v_ang
                twist.linear.x = 0.0
                twist.linear.y = 0.0
        else:
            # Not rotating: move in current_dir
            if self.current_dir == "forward":
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
            elif self.current_dir == "backward":
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
            elif self.current_dir == "left":
                twist.linear.x = 0.0
                twist.linear.y = self.v_strafe
            elif self.current_dir == "right":
                twist.linear.x = 0.0
                twist.linear.y = -self.v_strafe

            # Check how long we have moved in this direction
            elapsed_move = now - self.dir_start_time
            if elapsed_move >= self.move_duration:
                # Decide rotation direction based on current_dir
                if self.current_dir in ["backward", "left"]:
                    # Rotate LEFT 90°
                    self.rotation_dir = 1.0
                    self.get_logger().info(
                        f"Moved {self.current_dir} for {elapsed_move:.1f}s → rotate LEFT 90°"
                    )
                elif self.current_dir == "right":
                    # Rotate RIGHT 90°
                    self.rotation_dir = -1.0
                    self.get_logger().info(
                        f"Moved right for {elapsed_move:.1f}s → rotate RIGHT 90°"
                    )
                else:
                    # If forward, you can choose to not rotate or define your own rule
                    self.rotation_dir = 1.0
                    self.get_logger().info(
                        f"Moved forward for {elapsed_move:.1f}s → rotate LEFT 90° (default)"
                    )

                self.rotating = True
                self.rotation_start_time = now
                # Reset direction timer (after rotation you can change current_dir if desired)
                self.dir_start_time = now

        # Publish and store
        self.cmd = twist
        self.cmd_pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Rotate90Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
