import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerHolonomic(Node):
    def __init__(self):
        super().__init__('wall_follower_holonomic_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.3)    # desired distance to walls
        self.declare_parameter('forward_speed', 0.10)    # nominal forward speed
        self.declare_parameter('strafe_speed', 0.10)     # sideways speed (|vy|)
        self.declare_parameter('turn_speed', 0.40)       # angular speed (if needed)
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # not used for holonomic rules now

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_strafe = float(self.get_parameter('strafe_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()
        # Track last main movement direction: "forward", "backward", "left", "right"
        self.last_direction = "forward"
        # Flag: currently in "searching for wall" mode (no close wall)
        self.searching = False
        self.search_start_time = 0.0

        # Rotation and close-wall direction timer
        self.rotating = False                  # True when rotating to re-align
        self.rotation_dir = 1.0                # +1 = left, -1 = right
        self.close_dir = None                  # direction while close to wall
        self.close_dir_start_time = 0.0        # time when current close_dir started

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)  # 10 Hz

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower HOLONOMIC – vx, vy, w with full left/right handling."
        )

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
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
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

        # Dictionary of all regions and their minima
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

        # Find region with the smallest distance
        closest_region = min(region_mins, key=region_mins.get)
        closest_dist = region_mins[closest_region]

        now = self.get_clock().now().nanoseconds * 1e-9

        # ------------------------------------------------------------
        # 1) ROTATION MODE: rotate until wall is closest on target side
        # ------------------------------------------------------------
        if self.rotating:
            # If rotating LEFT (rotation_dir > 0) → want wall on RIGHT
            # If rotating RIGHT (rotation_dir < 0) → want wall on LEFT
            target_region = "RIGHT" if self.rotation_dir > 0 else "LEFT"

            if closest_region == target_region:
                # Stop rotating and resume normal wall following
                self.rotating = False
                action = (
                    f"Rotation done → wall closest on {target_region} at {closest_dist:.2f} m"
                )
                # After finishing, do normal close-wall behavior below
            else:
                # Keep rotating in chosen direction
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = self.rotation_dir * self.v_ang
                side_str = "LEFT" if self.rotation_dir > 0 else "RIGHT"
                action = f"Rotating {side_str} to align wall on the {target_region.lower()}"
                self.cmd = twist

                if action != self._last_action_logged:
                    self.get_logger().info(action)
                    self._last_action_logged = action
                self._state_action = action
                return  # do not run search/normal logic in this callback

        # ------------------------------------------------------------
        # 2) SEARCH MODE: same as previous version (no close wall)
        # ------------------------------------------------------------
        if closest_dist == float('inf') or closest_dist > self.base_distance:
            # No close wall → use the search behavior
            if not self.searching:
                # just transitioned to "no close wall"
                self.searching = True
                self.search_start_time = now

                if self.last_direction == "backward":
                    # move left
                    self.cmd.linear.x = 0.0
                    self.cmd.linear.y = self.v_strafe
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was BACKWARD → start moving LEFT"
                elif self.last_direction == "left":
                    # move forward
                    self.cmd.linear.x = self.v_lin
                    self.cmd.linear.y = 0.0
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was LEFT → start moving FORWARD"
                elif self.last_direction == "forward":
                    # move right
                    self.cmd.linear.x = 0.0
                    self.cmd.linear.y = -self.v_strafe
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was FORWARD → start moving RIGHT"
                elif self.last_direction == "right":
                    # move backward
                    self.cmd.linear.x = -self.v_lin
                    self.cmd.linear.y = 0.0
                    self.cmd.angular.z = 0.0
                    action = "No close wall → last was RIGHT → start moving BACKWARD"
                else:
                    # default if unknown
                    self.cmd.linear.x = self.v_lin
                    self.cmd.linear.y = 0.0
                    self.cmd.angular.z = 0.0
                    action = "No close wall → unknown last → start moving FORWARD"
            else:
                # already searching, keep same cmd
                action = "No close wall → keep searching movement"

            twist = self.cmd
            # When searching we do NOT track the 3s condition (only when close to wall)
            self.searching = True
            self.close_dir = None  # not in close-wall tracking
        else:
            # --------------------------------------------------------
            # 3) NORMAL CLOSE-WALL MODE: wall-follow + 3s direction check
            # --------------------------------------------------------
            self.searching = False  # reset search

            # Holonomic behaviors based on closest region (same as before)
            if closest_region == "FRONT":
                # When minimum distance is in the front side → move LEFT
                twist.linear.x = 0.0
                twist.linear.y = self.v_strafe
                twist.angular.z = 0.0
                action = f"FRONT {closest_dist:.2f} m → move LEFT (vy>0)"

            elif closest_region == "FR_RIGHT":
                # When minimum distance is in front-right → move front-left
                twist.linear.x = self.v_lin
                twist.linear.y = self.v_strafe
                twist.angular.z = 0.0
                action = f"FRONT-RIGHT {closest_dist:.2f} m → move FRONT-LEFT"

            elif closest_region == "RIGHT":
                # When minimum distance is in right → move forward parallel to wall
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = f"RIGHT {closest_dist:.2f} m → move FORWARD (parallel to right wall)"

            elif closest_region == "BACK_RIGHT":
                # When minimum distance is in back-right → move front-right
                twist.linear.x = self.v_lin
                twist.linear.y = -self.v_strafe
                twist.angular.z = 0.0
                action = f"BACK-RIGHT {closest_dist:.2f} m → move FRONT-RIGHT"

            elif closest_region == "BACK":
                # When minimum distance is in back → move RIGHT
                twist.linear.x = 0.0
                twist.linear.y = -self.v_strafe
                twist.angular.z = 0.0
                action = f"BACK {closest_dist:.2f} m → move RIGHT (vy<0)"

            # Left side behaviors
            elif closest_region == "FR_LEFT":
                # Wall front-left → move back-left (up in y, negative x)
                twist.linear.x = -self.v_lin
                twist.linear.y = self.v_strafe
                twist.angular.z = 0.0
                action = f"FRONT-LEFT {closest_dist:.2f} m → move BACK-LEFT"

            elif closest_region == "LEFT":
                # Wall directly on left → move DOWN/right (negative y)
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = f"LEFT {closest_dist:.2f} m → move DOWN (vy<0)"

            elif closest_region == "BACK_LEFT":
                # Wall back-left → move back-right (down in y, negative x)
                twist.linear.x = -self.v_lin
                twist.linear.y = -self.v_strafe
                twist.angular.z = 0.0
                action = f"BACK-LEFT {closest_dist:.2f} m → move BACK-RIGHT"

            # 3a) Track direction while close to wall
            dir_key = None
            if twist.linear.x > 0 and abs(twist.linear.y) < 1e-3:
                dir_key = "forward"
            elif twist.linear.x < 0 and abs(twist.linear.y) < 1e-3:
                dir_key = "backward"
            elif twist.linear.y > 0 and abs(twist.linear.x) < 1e-3:
                dir_key = "left"
            elif twist.linear.y < 0 and abs(twist.linear.x) < 1e-3:
                dir_key = "right"

            if dir_key is not None:
                if self.close_dir != dir_key:
                    # Direction changed (or first time)
                    self.close_dir = dir_key
                    self.close_dir_start_time = now
                else:
                    # Same direction as before → check duration
                    elapsed_close = now - self.close_dir_start_time
                    if elapsed_close >= 3.0:
                        if twist.linear.x > 0:
                            # forward → no rotation
                            pass
                        elif twist.linear.x < 0 or twist.linear.y > 0:
                            # backward or left → rotate LEFT
                            self.rotating = True
                            self.rotation_dir = 1.0
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.linear.y = 0.0
                            twist.angular.z = self.v_ang
                            action = (
                                f"Close-wall direction '{dir_key}' for 3s → start rotating LEFT"
                            )
                        elif twist.linear.y < 0:
                            # right → rotate RIGHT
                            self.rotating = True
                            self.rotation_dir = -1.0
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.linear.y = 0.0
                            twist.angular.z = -self.v_ang
                            action = (
                                f"Close-wall direction '{dir_key}' for 3s → start rotating RIGHT"
                            )

        # Update last commanded twist
        self.cmd = twist

        # Update last_direction based on current cmd (used for search mode)
        if self.cmd.linear.x > 0 and abs(self.cmd.linear.y) < 1e-3:
            self.last_direction = "forward"
        elif self.cmd.linear.x < 0 and abs(self.cmd.linear.y) < 1e-3:
            self.last_direction = "backward"
        elif self.cmd.linear.y > 0 and abs(self.cmd.linear.x) < 1e-3:
            self.last_direction = "left"
        elif self.cmd.linear.y < 0 and abs(self.cmd.linear.x) < 1e-3:
            self.last_direction = "right"

        # Logging (only on change)
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
