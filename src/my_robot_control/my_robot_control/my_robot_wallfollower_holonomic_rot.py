import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(qx, qy, qz, qw):
    """
    Converteix un quaternion a yaw (rotació al voltant de Z).
    Fórmula estàndard ZYX (roll-pitch-yaw).
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def normalize_angle(angle):
    """Normalitza un angle a (-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def shortest_angular_distance(from_angle, to_angle):
    """Distància angular mínima entre dos angles."""
    return normalize_angle(to_angle - from_angle)


class WallFollowerHolonomic(Node):
    def __init__(self):
        super().__init__('wall_follower_holonomic_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.4)    # desired distance to walls
        self.declare_parameter('forward_speed', 0.10)    # nominal forward speed
        self.declare_parameter('strafe_speed', 0.10)     # sideways speed (|vy|)
        self.declare_parameter('turn_speed', 0.40)       # angular speed (rad/s)
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # not used for holonomic rules now

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_strafe = float(self.get_parameter('strafe_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist
        self.cmd = Twist()
        self.last_direction = "forward"   # "forward", "backward", "left", "right"
        self.searching = False

        # Gir 90º a l'esquerra amb odom
        self.turning_left_90 = False
        self.turn_target_yaw = None
        self.turn_tolerance = math.radians(3.0)  # tolerància ~3º

        # Comptar 3 s de moviment pur cap a l'esquerra
        self.left_motion_start_time = None

        # Odometry
        self.current_yaw = 0.0
        self.odom_received = False

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 20)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)  # 10 Hz

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower HOLONOMIC – vx, vy, w + gir 90º esquerra amb odom (sense tf_transformations)."
        )

    # --------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        """Actualitza el yaw actual a partir del quaternion d'odom."""
        q = msg.pose.pose.orientation
        self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_received = True

    # --------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    # --------------------------------------------------------------------
    def stop(self):
        """Safe stop: zero cmd_vel and stop timers."""
        self._shutting_down = True

        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    # --------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # --------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # Mode gir 90º a l'esquerra: pren control exclusiu
        if self.turning_left_90:
            if not self.odom_received:
                twist = Twist()
                self.cmd = twist
                self._state_action = "Esperant odom per completar gir 90º"
                if self._last_action_logged != self._state_action:
                    self.get_logger().info(self._state_action)
                    self._last_action_logged = self._state_action
                return

            angle_err = shortest_angular_distance(self.current_yaw, self.turn_target_yaw)

            if abs(angle_err) <= self.turn_tolerance:
                # Hem arribat (± tolerància)
                self.turning_left_90 = False
                twist = Twist()
                self.cmd = twist
                self._state_action = "Acabat gir de 90º a l'esquerra, reprenc wall following."
                if self._last_action_logged != self._state_action:
                    self.get_logger().info(self._state_action)
                    self._last_action_logged = self._state_action
                return
            else:
                # Continuar girant cap a l'esquerra (CCW)
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = abs(self.v_ang)
                self.cmd = twist
                self._state_action = "Girant 90 graus a l'esquerra (amb odom)"
                if self._last_action_logged != self._state_action:
                    self.get_logger().info(self._state_action)
                    self._last_action_logged = self._state_action
                return

        # ----------------- Comportament normal wall follower -----------------
        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Right side regions (negative angles)
        FRONT        = []
        FR_RIGHT     = []
        RIGHT        = []
        BACK_RIGHT   = []
        BACK         = []

        # Left side regions (positive angles)
        FR_LEFT      = []
        LEFT         = []
        BACK_LEFT    = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # Right side
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160:
                BACK.append(d)

            # Left side (mirror)
            elif 20 < ang <= 70:
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang <= 160:
                BACK_LEFT.append(d)
            elif ang >= 160:
                BACK.append(d)  # also behind

        # Minimal distances per region
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')
        min_back_left  = min(BACK_LEFT)  if BACK_LEFT  else float('inf')

        twist = Twist()
        action = ""

        region_mins = {
            "FRONT":       min_front,
            "FR_RIGHT":    min_fr_right,
            "RIGHT":       min_right,
            "BACK_RIGHT":  min_back_right,
            "BACK":        min_back,
            "FR_LEFT":     min_fr_left,
            "LEFT":        min_left,
            "BACK_LEFT":   min_back_left,
        }

        closest_region = min(region_mins, key=region_mins.get)
        closest_dist = region_mins[closest_region]

        # Sense paret propera → moviment de cerca
        if closest_dist == float('inf') or closest_dist > self.base_distance:
            if not self.searching:
                self.searching = True

                if self.last_direction == "backward":
                    self.cmd.linear.x = 0.0
                    self.cmd.linear.y = self.v_strafe
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was BACKWARD → start moving LEFT"
                elif self.last_direction == "left":
                    self.cmd.linear.x = self.v_lin
                    self.cmd.linear.y = 0.0
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was LEFT → start moving FORWARD"
                elif self.last_direction == "forward":
                    self.cmd.linear.x = 0.0
                    self.cmd.linear.y = -self.v_strafe
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was FORWARD → start moving RIGHT"
                elif self.last_direction == "right":
                    self.cmd.linear.x = -self.v_lin
                    self.cmd.linear.y = 0.0
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was RIGHT → start moving BACKWARD"
                else:
                    self.cmd.linear.x = self.v_lin
                    self.cmd.linear.y = 0.0
                    self.cmd.angular.z = 0.0
                    action = "No close wall → unknown last → start moving FORWARD"
            else:
                action = "No close wall → keep searching movement"

            twist = self.cmd

        else:
            # Tenim paret → comportament holonòmic normal
            self.searching = False

            if closest_region == "FRONT":
                twist.linear.x = 0.0
                twist.linear.y = self.v_strafe
                twist.angular.z = 0.0
                action = f"FRONT {closest_dist:.2f} m → move LEFT (vy>0)"

            elif closest_region == "FR_RIGHT":
                twist.linear.x = self.v_lin
                twist.linear.y = self.v_strafe
                twist.angular.z = 0.0
                action = f"FRONT-RIGHT {closest_dist:.2f} m → move FRONT-LEFT"

            elif closest_region == "RIGHT":
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = f"RIGHT {closest_dist:.2f} m → move FORWARD (parallel to right wall)"

            elif closest_region == "BACK_RIGHT":
                twist.linear.x = self.v_lin
                twist.linear.y = -self.v_strafe
                twist.angular.z = 0.0
                action = f"BACK-RIGHT {closest_dist:.2f} m → move FRONT-RIGHT"

            elif closest_region == "BACK":
                twist.linear.x = 0.0
                twist.linear.y = -self.v_strafe
                twist.angular.z = 0.0
                action = f"BACK {closest_dist:.2f} m → move RIGHT (vy<0)"

            elif closest_region == "FR_LEFT":
                twist.linear.x = -self.v_lin
                twist.linear.y = self.v_strafe
                twist.angular.z = 0.0
                action = f"FRONT-LEFT {closest_dist:.2f} m → move BACK-LEFT"

            elif closest_region == "LEFT":
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = f"LEFT {closest_dist:.2f} m → move DOWN (vy<0)"

            elif closest_region == "BACK_LEFT":
                twist.linear.x = -self.v_lin
                twist.linear.y = -self.v_strafe
                twist.angular.z = 0.0
                action = f"BACK-LEFT {closest_dist:.2f} m → move BACK-RIGHT"

        # Actualitza cmd
        self.cmd = twist

        # Detectar 3 s de moviment pur cap a l'esquerra (vy>0, vx≈0, w≈0)
        moving_pure_left = (
            self.cmd.linear.y > 0.0 and
            abs(self.cmd.linear.x) < 1e-3 and
            abs(self.cmd.angular.z) < 1e-3
        )

        if moving_pure_left and not self.turning_left_90:
            if self.left_motion_start_time is None:
                self.left_motion_start_time = now
            else:
                if now - self.left_motion_start_time >= 3.0 and self.odom_received:
                    self.turning_left_90 = True
                    self.turn_target_yaw = normalize_angle(self.current_yaw + math.pi / 2.0)
                    self.left_motion_start_time = None
                    self.get_logger().info(
                        "3s anant purament cap a l'esquerra → començo gir de 90º a l'esquerra (amb odom)."
                    )
        else:
            self.left_motion_start_time = None

        # Actualitzar last_direction
        if self.cmd.linear.x > 0 and abs(self.cmd.linear.y) < 1e-3:
            self.last_direction = "forward"
        elif self.cmd.linear.x < 0 and abs(self.cmd.linear.y) < 1e-3:
            self.last_direction = "backward"
        elif self.cmd.linear.y > 0 and abs(self.cmd.linear.x) < 1e-3:
            self.last_direction = "left"
        elif self.cmd.linear.y < 0 and abs(self.cmd.linear.x) < 1e-3:
            self.last_direction = "right"

        # Log només si canvia
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

    # --------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerHolonomic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
