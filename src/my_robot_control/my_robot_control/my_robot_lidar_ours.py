import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleRobotControl(Node):

    def __init__(self):
        super().__init__('simple_robot_control')

        # Parámetros
        self.declare_parameter('distance_limit', 0.3)  # 30 cm
        self._distance_limit = self.get_parameter('distance_limit').value

        # Publicador para cmd_vel
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscripción al LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10  # QoS depth
        )

        # Mensaje para mover el robot
        self._msg = Twist()
        self._msg.linear.x = 0.2  # Velocidad hacia adelante
        self._msg.angular.z = 0.0

    def laser_callback(self, scan):
        # Buscar la distancia en el frente (cerca de 0 grados)
        # El ángulo del frente es 0 grados, por lo que buscamos cerca de ese valor
        front_angle = len(scan.ranges) // 2  # Aproximadamente en el centro del scan
        distance_front = scan.ranges[front_angle]

        # Verificar si la distancia es menor o igual al límite
        if distance_front <= self._distance_limit:
            # Detener el robot
            self._msg.linear.x = 0.0
            self._msg.angular.z = 0.0
            self.get_logger().info(f"Obstáculo detectado a {distance_front:.2f} m, deteniendo el robot.")
        else:
            # Continuar moviéndose hacia adelante
            self._msg.linear.x = 0.2  # Velocidad hacia adelante
            self._msg.angular.z = 0.0

        # Publicar el mensaje
        self._cmdVel.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    robot = SimpleRobotControl()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()


if __name__ == '__main__':
    main()
