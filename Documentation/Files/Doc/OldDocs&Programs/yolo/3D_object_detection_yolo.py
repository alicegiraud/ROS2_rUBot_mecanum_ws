import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros
import geometry_msgs.msg

class ObjectDetectionOverlay(Node):
    def __init__(self):
        super().__init__('object_detection_overlay')

        # Subscribe to the camera image, camera info, and detection topics
        self.image_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/detector_node/detections', self.detection_callback, 10)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize other variables
        self.bridge = CvBridge()
        self.detections = None
        self.image = None
        self.depth_image = None  # Original depth image for 3D calculations
        self.camera_info = None  # Camera intrinsic parameters

    def camera_info_callback(self, msg):
        # Store the camera info for intrinsic parameters
        self.camera_info = msg

    def image_callback(self, msg):
        try:
            # Convert the depth image to a float32 format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Normalize the depth image to range 0-255 for visualization (optional)
            normalized_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            
            # Convert normalized depth image to 8-bit grayscale for displaying
            display_image = np.uint8(normalized_depth_image)

            # Convert grayscale to color to allow color drawings (bounding boxes)
            display_image = cv2.cvtColor(display_image, cv2.COLOR_GRAY2BGR)

            # Store the images
            self.depth_image = depth_image  # Original depth image for 3D calculations
            self.image = display_image  # Image for visualization

            # Process and display the image with detections overlayed
            if self.detections is not None:
                self.overlay_detections()

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

    def detection_callback(self, msg):
        # Filter detections based on confidence
        filtered_detections = []
        for detection in msg.detections:
            # Assuming the first result contains the hypothesis with the highest confidence
            if detection.results[0].hypothesis.score >= 0.80:  # 80% confidence threshold
                filtered_detections.append(detection)

        # Store the filtered detections
        self.detections = Detection2DArray(detections=filtered_detections)

    def overlay_detections(self):
        if self.image is None or self.camera_info is None or self.depth_image is None:
            return

        # Get the intrinsic parameters from camera_info
        fx = self.camera_info.k[0]  # Focal length in x
        fy = self.camera_info.k[4]  # Focal length in y
        cx = self.camera_info.k[2]  # Optical center in x
        cy = self.camera_info.k[5]  # Optical center in y

        # Create a copy of the image to draw on
        overlay_image = self.image.copy()

        for detection in self.detections.detections:
            bbox = detection.bbox
            results = detection.results[0]  # Assuming only one result per detection

            # Calculate the bounding box in pixel coordinates
            x_min = int(bbox.center.position.x - (bbox.size_x / 2))
            y_min = int(bbox.center.position.y - (bbox.size_y / 2))
            x_max = int(bbox.center.position.x + (bbox.size_x / 2))
            y_max = int(bbox.center.position.y + (bbox.size_y / 2))

            # Draw the bounding box
            cv2.rectangle(overlay_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Calculate the centroid of the bounding box
            centroid_x = int(bbox.center.position.x)
            centroid_y = int(bbox.center.position.y)

            # Draw the centroid
            cv2.circle(overlay_image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

            # Get the depth at the centroid from the original depth image
            depth_value = self.depth_image[centroid_y, centroid_x]  # Use depth_image, not display_image

            # If depth is valid (not NaN or infinity)
            if np.isscalar(depth_value) and not np.isnan(depth_value) and not np.isinf(depth_value):
                # Calculate the 3D position in the camera_depth_optical_frame
                z = depth_value
                x = (centroid_x - cx) * z / fx
                y = (centroid_y - cy) * z / fy

                # Publish TF at the detected object's position
                self.publish_tf(x, y, z, results.hypothesis.class_id)

                # Overlay the 3D position on the image
                label_3d = f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f} m"
                cv2.putText(overlay_image, label_3d, (x_min, y_max + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # Overlay the detected object's name and confidence level
            object_name = results.hypothesis.class_id
            confidence = results.hypothesis.score * 100  # Confidence in percentage
            label = f"{object_name}: {confidence:.2f}%"

            # Put the label above the bounding box
            cv2.putText(overlay_image, label, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the image with OpenCV
        cv2.imshow("Detections Overlay", overlay_image)
        cv2.waitKey(1)

    def publish_tf(self, x, y, z, object_name):
        # Create a TransformStamped message
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_depth_optical_frame"  # Parent frame
        t.child_frame_id = f"{object_name}_frame"  # Child frame for the detected object
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # Default rotation (no rotation)

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()