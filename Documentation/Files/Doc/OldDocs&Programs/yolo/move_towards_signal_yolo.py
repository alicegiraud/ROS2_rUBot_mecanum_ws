import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import math

class MoveTowardsObject(Node):
    def __init__(self):
        super().__init__('move_towards_object')

        # Create a publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to periodically check the transform and move the robot
        self.timer = self.create_timer(0.1, self.control_loop)

        # Define the target frame that you want to move towards
        self.target_frame = 'person_frame'  # Replace with the actual TF child frame name

    def control_loop(self):
        try:
            # Look up the latest transform from the target object to the camera frame
            trans = self.tf_buffer.lookup_transform('camera_depth_optical_frame', self.target_frame, rclpy.time.Time())

            # Extract the translation components (x, y, z)
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Calculate distance and angle to the target
            distance = math.sqrt(x**2 + y**2)
            angle_to_target = math.atan2(y, x)

            # Define a Twist message to move the robot
            twist = Twist()

            print("[DEBUG] Angle to target: ", angle_to_target)
            print("[DEBUG] Distance to target: ", distance)

            # Move forward if the target is not too close
            if distance > 0.04:  # Stop if we're within 0.5 meters of the target
                twist.linear.x = 0.2 * distance  # Scale speed by distance
                twist.angular.z = 0.1 * angle_to_target  # Turn towards the target
            else:
                print("Too close to person !")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # Publish the Twist message to move the robot
            self.velocity_publisher.publish(twist)

        except tf2_ros.LookupException:
            self.get_logger().warn(f"Could not find transform from camera_depth_optical_frame to {self.target_frame}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Extrapolation error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveTowardsObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
