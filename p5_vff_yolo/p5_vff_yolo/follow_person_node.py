import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from enum import IntEnum

from geometry_msgs.msg import Twist, Vector3, PointStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point  # noqa: F401 (registers TF for PointStamped)


class State(IntEnum):
    SEARCHING = 0
    FOLLOWING = 1


class FollowPersonNode(Node):
    """FSM node: SEARCHING (rotate) <-> FOLLOWING (VFF toward person)."""

    def __init__(self):
        super().__init__('follow_person_node')

        # Parametros configurables desde launch
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('optical_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('search_angular_speed', 0.3)
        self.declare_parameter('person_lost_timeout', 1.0)
        self.declare_parameter('repulsive_gain_factor', 1.0)
        self.declare_parameter('repulsive_influence_distance', 0.5)

        self.target_class = self.get_parameter('target_class').value
        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value
        self.person_lost_timeout = self.get_parameter('person_lost_timeout').value
        self.repulsive_gain_factor = self.get_parameter('repulsive_gain_factor').value
        self.repulsive_influence_distance = self.get_parameter(
            'repulsive_influence_distance').value

        # TF2 para transformar deteccion de frame optico a base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Intrinsecos de camara (fx, cx) - se obtienen del primer CameraInfo
        self.camera_configured = False
        self.f_x = 0.0
        self.c_x = 0.0

        # Estado FSM
        self.state = State.SEARCHING
        self.last_person_time = self.get_clock().now()

        # Vectores VFF
        self.attractive_vec = None
        self.repulsive_vec = Vector3()

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera_info',
            self.camera_info_callback, rclpy.qos.qos_profile_sensor_data)

        self.detection_sub = self.create_subscription(
            Detection2DArray, 'input_detection_2d',
            self.detection_callback, rclpy.qos.qos_profile_sensor_data)

        self.repulsive_sub = self.create_subscription(
            Vector3, 'repulsive_vector',
            self.repulsive_callback, 10)

        # Publisher de velocidad
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Ciclo de control a 20 Hz
        self.timer = self.create_timer(0.05, self.control_cycle)

        self.get_logger().info(
            f'FollowPersonNode started — target_class={self.target_class}')

    # -- Callbacks ----------------------------------------------------------

    def camera_info_callback(self, msg):
        """Obtiene fx y cx de la matriz K (solo primer mensaje)."""
        self.f_x = msg.k[0]
        self.c_x = msg.k[2]
        self.camera_configured = True
        self.get_logger().info(
            f'Camera intrinsics received: fx={self.f_x:.2f}, cx={self.c_x:.2f}')
        self.destroy_subscription(self.camera_info_sub)

    def detection_callback(self, msg):
        """Filtra detecciones YOLO: elige la persona con bbox mas grande."""
        if not msg.detections or not self.camera_configured:
            return

        largest = None
        largest_area = 0.0

        for detection in msg.detections:
            if not detection.results:
                continue
            if detection.results[0].hypothesis.class_id != self.target_class:
                continue
            area = detection.bbox.size_x * detection.bbox.size_y
            if area > largest_area:
                largest_area = area
                largest = detection

        if largest is None:
            return

        # Solo transiciona a FOLLOWING si la TF es exitosa
        if self.compute_attractive_vector(largest):
            self.last_person_time = self.get_clock().now()
            if self.state == State.SEARCHING:
                self.go_state(State.FOLLOWING)

    def repulsive_callback(self, msg):
        """Recibe vector repulsivo del obstacle_detector_node."""
        self.repulsive_vec = msg

    # -- Vector atractivo (deteccion 2D -> angulo -> TF -> base_footprint) --

    def compute_attractive_vector(self, detection):
        """Calcula vector atractivo desde bbox 2D usando intrinsecos + TF."""
        # Angulo horizontal desde el centro de la imagen
        x_pixel = detection.bbox.center.position.x
        pixel_offset = x_pixel - self.c_x
        angle = math.atan(pixel_offset / self.f_x)

        # Punto en frame optico (z=1m fijo, sin profundidad en 2D)
        target_point = PointStamped()
        target_point.header = detection.header
        target_point.point.x = math.tan(angle)
        target_point.point.y = 0.0
        target_point.point.z = 1.0

        # Transformar de frame optico a base_footprint
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame,
                detection.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5))
            pt_base = do_transform_point(target_point, transform)
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}', throttle_duration_sec=2.0)
            return False

        vec = Vector3()
        vec.x = pt_base.point.x
        vec.y = pt_base.point.y
        vec.z = 0.0
        self.attractive_vec = vec

        angle_base = math.atan2(vec.y, vec.x)
        self.get_logger().info(
            f'Person at {math.degrees(angle_base):.1f} deg '
            f'(area={detection.bbox.size_x * detection.bbox.size_y:.0f} px)',
            throttle_duration_sec=0.5)
        return True

    # -- VFF: attractive - repulsive ----------------------------------------

    def compute_vff(self):
        """Combina vector atractivo y repulsivo para generar velocidad."""
        cmd = Twist()
        if self.attractive_vec is None:
            return cmd

        # Fuerza repulsiva: 1/d^2, solo dentro de rho_0
        obstacle_dist = math.hypot(self.repulsive_vec.x, self.repulsive_vec.y)
        rep_force_x = 0.0
        rep_force_y = 0.0

        rho_0 = self.repulsive_influence_distance
        if 0.0 < obstacle_dist <= rho_0:
            force_mag = 1.0 / obstacle_dist ** 2
            unit_x = self.repulsive_vec.x / obstacle_dist
            unit_y = self.repulsive_vec.y / obstacle_dist
            rep_force_x = self.repulsive_gain_factor * force_mag * unit_x
            rep_force_y = self.repulsive_gain_factor * force_mag * unit_y

        # VFF = atractivo - repulsivo
        vff_x = self.attractive_vec.x - rep_force_x
        vff_y = self.attractive_vec.y - rep_force_y

        angle = math.atan2(vff_y, vff_x)
        cmd.linear.x = min(self.max_linear_speed, math.hypot(vff_x, vff_y))
        rotation_dir = 1.0 if angle >= 0.0 else -1.0
        cmd.angular.z = rotation_dir * min(self.max_angular_speed, abs(angle))

        self.get_logger().debug(
            f'VFF ({vff_x:.2f}, {vff_y:.2f}) -> '
            f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}')
        return cmd

    # -- FSM: ciclo de control a 20 Hz -------------------------------------

    def control_cycle(self):
        cmd = Twist()

        if self.state == State.SEARCHING:
            cmd.angular.z = self.search_angular_speed
            self.get_logger().info(
                'SEARCHING — rotating...', throttle_duration_sec=3.0)

        elif self.state == State.FOLLOWING:
            elapsed = (self.get_clock().now() - self.last_person_time)
            if elapsed > Duration(seconds=self.person_lost_timeout):
                # Persona perdida -> volver a buscar
                self.go_state(State.SEARCHING)
                self.attractive_vec = None
                cmd.angular.z = self.search_angular_speed
            else:
                cmd = self.compute_vff()

        self.vel_pub.publish(cmd)
        self.repulsive_vec = Vector3()  # Reset: sin mensaje nuevo = sin obstaculo

    # -- Transiciones -------------------------------------------------------

    def go_state(self, new_state):
        self.state = new_state
        self.get_logger().info(f'State -> {new_state.name}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
