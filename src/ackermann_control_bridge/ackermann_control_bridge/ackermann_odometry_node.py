#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu

import math
import numpy as np
from typing import Tuple, Optional


class AckermannOdometryNode(Node):
    def __init__(self):
        super().__init__('ackermann_odometry_node')

        # Dichiarazione parametri del robot
        self.declare_parameter('wheelbase', 0.4)  # Distanza tra asse anteriore e posteriore
        self.declare_parameter('track_width', 0.3)  # Distanza tra i centri delle ruote su un asse
        self.declare_parameter('wheel_radius', 0.05)  # Raggio delle ruote
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('footprint_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)  # Frequenza di pubblicazione (Hz)

        # Nomi dei giunti (da configurare per il tuo robot)
        self.declare_parameter('rear_left_wheel_joint_name', 'rear_left_wheel_joint')
        self.declare_parameter('rear_right_wheel_joint_name', 'rear_right_wheel_joint')
        self.declare_parameter('front_left_steer_joint_name', 'front_left_steer_joint')
        self.declare_parameter('front_right_steer_joint_name', 'front_right_steer_joint')

        # Pesi per la fusione dell'odometria (0-1)
        self.declare_parameter('imu_orientation_weight', 0.7)
        self.declare_parameter('wheel_encoder_weight', 0.7)
        self.declare_parameter('cmd_vel_weight', 0.1)  # Peso più basso per cmd_vel

        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.footprint_frame = self.get_parameter('footprint_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_orientation_weight = self.get_parameter('imu_orientation_weight').value
        self.wheel_encoder_weight = self.get_parameter('wheel_encoder_weight').value
        self.cmd_vel_weight = self.get_parameter('cmd_vel_weight').value

        self.rear_left_wheel_joint_name = self.get_parameter('rear_left_wheel_joint_name').value
        self.rear_right_wheel_joint_name = self.get_parameter('rear_right_wheel_joint_name').value
        self.front_left_steer_joint_name = self.get_parameter('front_left_steer_joint_name').value
        self.front_right_steer_joint_name = self.get_parameter('front_right_steer_joint_name').value

        # Inizializzazione stato odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0  # Rimarrà 0 per Ackermann
        self.vtheta = 0.0

        # Ultimi dati sensore noti per il calcolo dell'odometria
        self.last_joint_states_time: Optional[rclpy.time.Time] = None
        self.last_imu_yaw: Optional[float] = None
        self.last_cmd_vel_time: Optional[rclpy.time.Time] = None
        self.last_cmd_linear_x: float = 0.0
        self.last_cmd_angular_z: float = 0.0

        # Valori dai Joint States
        self.rear_left_wheel_vel: float = 0.0
        self.rear_right_wheel_vel: float = 0.0
        self.front_left_steer_angle: float = 0.0
        self.front_right_steer_angle: float = 0.0

        # TF broadcaster per la trasformata odom -> base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher per il messaggio Odometry
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        )

        # Iscrizioni ai topic
        qos_profile_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile_sensor_data
        )

        # Timer per la pubblicazione dell'odometria
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom_and_update_pose)
        self.get_logger().info('Ackermann Odometry Node has started!')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_linear_x = msg.linear.x
        self.last_cmd_angular_z = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState):
        # Estrai velocità delle ruote motrici e angoli di sterzo
        for i, name in enumerate(msg.name):
            if name == self.rear_left_wheel_joint_name and i < len(msg.velocity):
                self.rear_left_wheel_vel = msg.velocity[i]
            elif name == self.rear_right_wheel_joint_name and i < len(msg.velocity):
                self.rear_right_wheel_vel = msg.velocity[i]
            elif name == self.front_left_steer_joint_name and i < len(msg.position):
                self.front_left_steer_angle = msg.position[i]
            elif name == self.front_right_steer_joint_name and i < len(msg.position):
                self.front_right_steer_angle = msg.position[i]

        self.last_joint_states_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        # Estrai lo yaw dalla quaternione IMU
        _, _, imu_yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.last_imu_yaw = imu_yaw

    def publish_odom_and_update_pose(self):
        current_time = self.get_clock().now()

        # Calcolo dt basato sull'ultima lettura valida di joint_states
        if self.last_joint_states_time is None:
            # Non abbiamo ancora ricevuto dati da joint_states, non possiamo calcolare l'odometria da encoder/sterzo
            # Potremmo usare solo cmd_vel qui, ma per precisione aspettiamo i sensori reali.
            # Alternativa: se vogliamo che il robot si muova immediatamente con cmd_vel anche senza encoder,
            # potremmo usare self.vx = self.last_cmd_linear_x * self.cmd_vel_weight
            # e self.vtheta = self.last_cmd_angular_z * self.cmd_vel_weight
            # con i dovuti pesi e normalizzazioni.
            self.get_logger().warn("Waiting for first /joint_states message to calculate odometry.")
            return

        dt = (current_time - (
                    self.last_joint_states_time or current_time)).nanoseconds / 1e9  # Usa last_joint_states_time se disponibile, altrimenti current_time

        if dt <= 0:
            return

        # --- Calcolo di Vx e Vtheta da Encoder Ruote e Angoli di Sterzo (Modello Ackermann) ---
        linear_x_encoder_model = 0.0
        angular_z_encoder_model = 0.0

        # Velocità lineare dal feedback delle ruote posteriori
        wheel_avg_velocity = (self.rear_left_wheel_vel + self.rear_right_wheel_vel) / 2.0
        linear_x_encoder_model = wheel_avg_velocity * self.wheel_radius

        # Velocità angolare dal modello a bicicletta (usa angolo di sterzo medio)
        # Protezione per NaN e divisione per zero
        if math.isnan(self.front_left_steer_angle) or math.isnan(self.front_right_steer_angle):
            self.get_logger().warn(
                "NaN detected in steering angles from JointState. Using zero angular velocity from model.")
            angular_z_encoder_model = 0.0
        else:
            steer_angle_avg = (self.front_left_steer_angle + self.front_right_steer_angle) / 2.0

            if abs(steer_angle_avg) < 1e-4:  # Piccolissimo angolo di sterzo, assumiamo movimento dritto
                angular_z_encoder_model = 0.0
            else:
                try:
                    # v_theta = v_linear * tan(steer_angle) / wheelbase
                    angular_z_encoder_model = linear_x_encoder_model * math.tan(steer_angle_avg) / self.wheelbase
                except ZeroDivisionError:
                    self.get_logger().warn(
                        "Division by zero in angular velocity calculation (wheelbase is zero?). Using zero angular velocity from model.")
                    angular_z_encoder_model = 0.0
                except ValueError:  # math.tan of very large angle (close to pi/2)
                    self.get_logger().warn(
                        "Invalid steering angle for tan calculation. Using zero angular velocity from model.")
                    angular_z_encoder_model = 0.0

        # --- Fusione delle Velocità ---
        fused_vx = (linear_x_encoder_model * self.wheel_encoder_weight)
        fused_vtheta = (angular_z_encoder_model * self.wheel_encoder_weight)

        total_weight_linear = self.wheel_encoder_weight
        total_weight_angular = self.wheel_encoder_weight

        # Fonde con cmd_vel se recente
        if self.last_cmd_vel_time is not None and (current_time - self.last_cmd_vel_time).nanoseconds / 1e9 < 0.5:
            fused_vx += (self.last_cmd_linear_x * self.cmd_vel_weight)
            fused_vtheta += (
                        self.last_cmd_angular_z * self.cmd_vel_weight)  # cmd_vel angular is often absolute for robot base
            total_weight_linear += self.cmd_vel_weight
            total_weight_angular += self.cmd_vel_weight

        # Fonde con IMU per l'orientamento (se disponibile e se il peso IMU è alto, l'IMU può dominare l'angolo assoluto)
        # Per la velocità angolare, l'IMU può fornire una stima più diretta di vtheta.
        # Questa fusione è per la posa, non direttamente per vtheta qui, vtheta è calcolata dal modello.
        # Per vtheta, se l'IMU fornisce anche velocità angolare, potresti fonderla qui.
        # Per ora, la fusione IMU è più per la correzione della posa theta.

        if total_weight_linear > 0:
            self.vx = fused_vx / total_weight_linear
        else:
            self.vx = 0.0  # Se non ci sono input validi, la velocità lineare è zero

        if total_weight_angular > 0:
            self.vtheta = fused_vtheta / total_weight_angular
        else:
            self.vtheta = 0.0  # Se non ci sono input validi, la velocità angolare è zero

        # --- Aggiornamento della Posa ---
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Ensure last_time is initialized
        if not hasattr(self, 'last_time'):
            self.last_time = current_time

        # Update theta using angular velocity from cmd_vel
        delta_theta = self.last_cmd_angular_z * (current_time - self.last_time).nanoseconds * 1e-9
        self.theta += delta_theta
        self.theta = math.fmod(self.theta + math.pi, 2 * math.pi) - math.pi  # Normalize to [-π, π]

        # Update last_time
        self.last_time = current_time

        # Fusione dello yaw IMU per la correzione della posa assoluta
        # Questo sovrascrive parzialmente lo yaw calcolato dall'odometria
        if self.last_imu_yaw is not None and self.imu_orientation_weight > 0:
            # Per una fusione più robusta, è meglio usare un filtro (es. EKF).
            # Qui si applica una media ponderata diretta.
            self.theta = (self.theta * (1.0 - self.imu_orientation_weight)) + (
                        self.last_imu_yaw * self.imu_orientation_weight)

        self.theta = math.fmod(self.theta, 2 * math.pi)  # Normalizza l'angolo

        # --- Pubblicazione delle Trasformate e del Messaggio Odometry ---
        self.publish_odom_tf_and_msg(current_time)

        # Debugging logs:
        self.get_logger().debug(f'--- Debug Odometria ---')
        self.get_logger().debug(f'Encoder Model: Vx={linear_x_encoder_model:.2f}, Vtheta={angular_z_encoder_model:.2f}')
        self.get_logger().debug(
            f'Last Cmd_vel: linear_x={self.last_cmd_linear_x:.2f}, angular_z={self.last_cmd_angular_z:.2f}')
        self.get_logger().debug(f'Fused Vel: vx={self.vx:.2f}, vtheta={self.vtheta:.2f}')
        self.get_logger().debug(f'Current Odom Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
        self.get_logger().debug(f'-----------------------')

    def publish_odom_tf_and_msg(self, current_time: Time):
        # Pubblica la trasformata (odom -> base_footprint)
        odom_tf = TransformStamped()
        odom_tf.header.stamp = current_time.to_msg()
        odom_tf.header.frame_id = self.odom_frame
        odom_tf.child_frame_id = self.footprint_frame
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0  # Si assume robot 2D
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_tf.transform.rotation.x = qx
        odom_tf.transform.rotation.y = qy
        odom_tf.transform.rotation.z = qz
        odom_tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(odom_tf)

        # Pubblica il messaggio Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.footprint_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        # Aggiungi la covarianza (diagonale semplice per ora, regola in base al rumore dei sensori)
        odom_msg.pose.covariance = np.diag([
            0.05, 0.05, 0.0, 0.0, 0.0, 0.03  # x, y, z, roll, pitch, yaw
        ]).flatten().tolist()
        odom_msg.twist.covariance = np.diag([
            0.05, 0.05, 0.0, 0.0, 0.0, 0.03  # vx, vy, vz, vroll, vpitch, vyaw
        ]).flatten().tolist()

        self.odom_publisher.publish(odom_msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw) -> Tuple[float, float, float, float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    @staticmethod
    def quaternion_to_euler(x, y, z, w) -> Tuple[float, float, float]:
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()