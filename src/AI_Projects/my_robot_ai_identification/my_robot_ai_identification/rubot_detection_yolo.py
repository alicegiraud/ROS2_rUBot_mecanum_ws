#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference

from ament_index_python.packages import get_package_share_directory
import os
import math

import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tf_transformations import euler_from_quaternion, quaternion_from_euler


class YoloObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.declare_parameter('use_sim_time', False) # Allow sim time when set from launch
        # ------------------- Parameters -------------------
        self.declare_parameter('modelYolo', 'yolov8n_custom.pt')
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('front_distance', 1.0)
        self.declare_parameter('sign_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')  # or base_footprint
        self.declare_parameter('sign_positions', {})  # dict: {SignName: [x, y]}

        # Hold times (seconds)
        self.declare_parameter('hold_stop_s', 3.0)
        self.declare_parameter('hold_prohibido_s', 5.0)
        self.declare_parameter('hold_ceda_s', 2.0)
        self.declare_parameter('cooldown_repeat_s', 5.0)  # avoid repeated triggers

        # Waypoint offsets (meters)
        self.declare_parameter('wp_forward_m', 0.8)   # go forward after sign (STOP/Ceda)
        self.declare_parameter('wp_lateral_m', 0.65)  # lateral bypass (Prohibido / turns)

        # Read parameters
        model_file = self.get_parameter('modelYolo').value
        self.image_topic = self.get_parameter('topic').value
        self.front_distance = float(self.get_parameter('front_distance').value)
        self.sign_frame = self.get_parameter('sign_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.hold_stop_s = float(self.get_parameter('hold_stop_s').value)
        self.hold_prohibido_s = float(self.get_parameter('hold_prohibido_s').value)
        self.hold_ceda_s = float(self.get_parameter('hold_ceda_s').value)
        self.cooldown_repeat_s = float(self.get_parameter('cooldown_repeat_s').value)

        self.wp_forward_m = float(self.get_parameter('wp_forward_m').value)
        self.wp_lateral_m = float(self.get_parameter('wp_lateral_m').value)

        raw_sign_positions = self.get_parameter('sign_positions').value
        if not isinstance(raw_sign_positions, dict):
            raise RuntimeError(f"sign_positions must be a dict. Got {type(raw_sign_positions)}")

        # Validate sign_positions: each value must be [x, y]
        self.sign_positions = {}
        for name, xy in raw_sign_positions.items():
            if (not isinstance(xy, (list, tuple))) or len(xy) != 2:
                self.get_logger().warn(f'Ignoring sign "{name}": expected [x,y], got {xy}')
                continue
            self.sign_positions[str(name)] = (float(xy[0]), float(xy[1]))

        # ------------------- TF (map -> base_link) -------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------- Model path (portable) -------------------
        package_path = get_package_share_directory('my_robot_ai_identification')
        self.model_path = os.path.join(package_path, 'models', model_file)

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        self.get_logger().info(f"Loaded YOLO model: {self.model_path}")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"front_distance: {self.front_distance} m")
        self.get_logger().info(f"sign_frame: {self.sign_frame} | base_frame: {self.base_frame}")
        self.get_logger().info(f"sign_positions: {self.sign_positions}")

        # ------------------- Setup -------------------
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.camera_callback,
            qos_profile_sensor_data
        )

        # Publishers for inference visualization (kept)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        # Publisher for traffic waypoints
        self.waypoint_pub = self.create_publisher(PoseStamped, "/traffic_waypoint", 10)

        # --- Odom debug subscriber (optional) ---
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ------------------- State for non-blocking waits -------------------
        self.hold_until = 0.0  # seconds (node time)
        self.last_trigger_time = {}  # sign_name -> last_trigger_time_s

    # ------------------------------------------------------------------
    # ODOM (debug)
    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_yaw = yaw

    # TF helpers
    def get_robot_xy_yaw_in_map(self):
        """
        Get robot pose in 'map' using TF.
        Short note: TF gives map->base_link, so sign positions in map are comparable.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.sign_frame,
                self.base_frame,
                rclpy.time.Time()  # latest
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed ({self.sign_frame} -> {self.base_frame}): {e}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return (x, y, yaw)

    def get_sign_distance(self, sign_name: str):
        """
        Distance computed in map frame.
        - Robot pose comes from TF in 'map'.
        - Sign positions are defined in 'map' (same frame).
        """
        if sign_name not in self.sign_positions:
            return None

        robot_pose = self.get_robot_xy_yaw_in_map()
        if robot_pose is None:
            return None

        rx, ry, _ = robot_pose
        sx, sy = self.sign_positions[sign_name]

        # Euclidean distance in the same frame ('map')
        return math.hypot(sx - rx, sy - ry)

    def should_react(self, sign_name: str) -> bool:
        d = self.get_sign_distance(sign_name)
        return (d is not None) and (d <= self.front_distance)

    # ------------------------------------------------------------------
    # Waypoint generation (coherent)
    def make_waypoint_relative_to_robot(self, dx_forward: float, dy_left: float, yaw: float):
        """
        Build a waypoint in map frame using robot yaw.
        dx_forward: +forward in robot frame
        dy_left:    +left in robot frame
        """
        robot_pose = self.get_robot_xy_yaw_in_map()
        if robot_pose is None:
            return None

        rx, ry, _ = robot_pose

        # Rotate (dx,dy) from robot frame to map frame
        wx = rx + dx_forward * math.cos(yaw) - dy_left * math.sin(yaw)
        wy = ry + dx_forward * math.sin(yaw) + dy_left * math.cos(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.sign_frame  # "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def make_waypoint_near_sign(self, sign_name: str, dx_forward: float, dy_left: float):
        """
        Waypoint centered around the SIGN position but oriented with robot yaw.
        This keeps behavior consistent even if signs are placed in map.
        """
        if sign_name not in self.sign_positions:
            return None

        robot_pose = self.get_robot_xy_yaw_in_map()
        if robot_pose is None:
            return None
        _, _, yaw = robot_pose

        sx, sy = self.sign_positions[sign_name]

        wx = sx + dx_forward * math.cos(yaw) - dy_left * math.sin(yaw)
        wy = sy + dx_forward * math.sin(yaw) + dy_left * math.cos(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.sign_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    # ------------------------------------------------------------------
    # CAMERA
    def camera_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)

        yolov8_msg = Yolov8Inference()
        yolov8_msg.header.frame_id = "inference"
        yolov8_msg.header.stamp = self.get_clock().now().to_msg()

        detected_signs = []
        for r in results:
            for box in r.boxes:
                inf = InferenceResult()
                b = box.xyxy[0].to('cpu').numpy()
                c = int(box.cls)
                class_name = self.model.names[c]

                detected_signs.append(class_name)

                inf.class_name = class_name
                inf.left, inf.top, inf.right, inf.bottom = map(int, b)
                inf.box_width = inf.right - inf.left
                inf.box_height = inf.bottom - inf.top
                inf.x = inf.left + inf.box_width / 2.0
                inf.y = inf.top + inf.box_height / 2.0
                yolov8_msg.yolov8_inference.append(inf)

        self.handle_signs(detected_signs)

        if results:
            annotated = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.img_pub.publish(img_msg)

        self.yolov8_pub.publish(yolov8_msg)

    # ------------------------------------------------------------------
    # Log POSE comparison
    def log_pose_comparison(self, sign_name: str, dist_map: float):
        # Robot pose in map from TF
        robot_pose_map = self.get_robot_xy_yaw_in_map()

        # Optional: map->odom transform (shows drift/correction)
        map_odom = None
        map_odom_yaw = None
        try:
            tf_mo = self.tf_buffer.lookup_transform(
                self.sign_frame,   # usually "map"
                "odom",
                rclpy.time.Time()
            )

            tx = tf_mo.transform.translation.x
            ty = tf_mo.transform.translation.y

            q = tf_mo.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            map_odom = (tx, ty)
            map_odom_yaw = yaw

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        if robot_pose_map is None:
            if self.odom_x is not None and self.odom_y is not None and self.odom_yaw is not None:
                odom_raw = f"({self.odom_x:.2f},{self.odom_y:.2f},{self.odom_yaw:.2f})"
            else:
                odom_raw = "(n/a)"

            self.get_logger().info(
                f"[SIGN] {sign_name} | dist(map)={dist_map:.2f} m | TF(map->{self.base_frame}) unavailable | "
                f"odom_pose={odom_raw}"
            )
            return

        mx, my, myaw = robot_pose_map

        # Print odom pose if available
        if self.odom_x is not None and self.odom_y is not None and self.odom_yaw is not None:
            odom_str = f"odom_pose=({self.odom_x:.2f},{self.odom_y:.2f},{self.odom_yaw:.2f})"
        else:
            odom_str = "odom_pose=(n/a)"

        if map_odom is not None and map_odom_yaw is not None:
            map_odom_str = (
                f"map->odom_trans=({map_odom[0]:.2f},{map_odom[1]:.2f}) "
                f"yaw={map_odom_yaw:.2f}"
            )
        else:
            map_odom_str = "map->odom_trans=(n/a)"

        self.get_logger().info(
            f"[SIGN] {sign_name} | dist(map)={dist_map:.2f} m | "
            f"map_pose(TF)=({mx:.2f},{my:.2f},{myaw:.2f}) | {odom_str} | {map_odom_str}"
        )

    # ------------------------------------------------------------------
    # SIGN LOGIC (non-blocking)
    def handle_signs(self, detected_signs):
        now = self.get_clock().now().nanoseconds / 1e9

        # If we are in a "hold" window, ignore new detections (non-blocking wait)
        if now < self.hold_until:
            return

        def can_trigger(sign_name: str) -> bool:
            last = self.last_trigger_time.get(sign_name, -1e9)
            return (now - last) >= self.cooldown_repeat_s

        # Priority order: Prohibido > STOP > Ceda > turns
        if "Prohibido" in detected_signs and self.should_react("Prohibido") and can_trigger("Prohibido"):
            dist = self.get_sign_distance("Prohibido") or 0.0
            self.log_pose_comparison("Prohibido", dist)
            self.get_logger().info(" → hold + bypass waypoint")

            # Hold time (non-blocking)
            self.hold_until = now + self.hold_prohibido_s
            self.last_trigger_time["Prohibido"] = now

            # Bypass: lateral move (choose left by default; change sign convention as needed)
            wp = self.make_waypoint_near_sign("Prohibido", dx_forward=self.wp_forward_m, dy_left=+self.wp_lateral_m)
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif "STOP" in detected_signs and self.should_react("STOP") and can_trigger("STOP"):
            dist = self.get_sign_distance("STOP") or 0.0
            self.log_pose_comparison("STOP", dist)
            self.get_logger().info(" → hold + forward waypoint")

            self.hold_until = now + self.hold_stop_s
            self.last_trigger_time["STOP"] = now

            # After STOP: go forward a bit
            wp = self.make_waypoint_near_sign("STOP", dx_forward=self.wp_forward_m, dy_left=0.0)
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif "Ceda" in detected_signs and self.should_react("Ceda") and can_trigger("Ceda"):
            dist = self.get_sign_distance("Ceda") or 0.0
            self.log_pose_comparison("Ceda", dist)
            self.get_logger().info(" → short hold + forward waypoint")

            self.hold_until = now + self.hold_ceda_s
            self.last_trigger_time["Ceda"] = now

            # Ceda: slight forward waypoint (no lateral)
            wp = self.make_waypoint_near_sign("Ceda", dx_forward=self.wp_forward_m, dy_left=0.0)
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif "Derecha" in detected_signs and self.should_react("Derecha") and can_trigger("Derecha"):
            dist = self.get_sign_distance("Derecha") or 0.0
            self.log_pose_comparison("Derecha", dist)
            self.get_logger().info(" → right waypoint")

            self.last_trigger_time["Derecha"] = now
            wp = self.make_waypoint_near_sign("Derecha", dx_forward=self.wp_forward_m, dy_left=-self.wp_lateral_m)
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif "Izquierda" in detected_signs and self.should_react("Izquierda") and can_trigger("Izquierda"):
            dist = self.get_sign_distance("Izquierda") or 0.0
            self.log_pose_comparison("Izquierda", dist)
            self.get_logger().info(" → left waypoint")

            self.last_trigger_time["Izquierda"] = now
            wp = self.make_waypoint_near_sign("Izquierda", dx_forward=self.wp_forward_m, dy_left=+self.wp_lateral_m)
            if wp is not None:
                self.waypoint_pub.publish(wp)


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
