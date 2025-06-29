import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import time


class EstadoMissao(Enum):
    EXPLORANDO = 1
    NAVIGANDO_PARA_BANDEIRA = 2
    POSICIONANDO_PARA_COLETA = 3
    CAPTURANDO_BANDEIRA = 4
    RETORNANDO_BASE = 5
    POSICIONANDO_PARA_DEPOSITO = 6
    DEPOSITANDO_BANDEIRA = 7


class ControleMissao(Node):
    def __init__(self):
        super().__init__('controle_missao')
        self.estado = EstadoMissao.EXPLORANDO

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.garra_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)

        self.bridge = CvBridge()

        self.obstaculo_frontal = False
        self.bandeira_visivel = False
        self.cx = None
        self.bandeira_ja_encontrada = False

        self.limite_obstaculo_frontal = 0.6
        self.pose_atual = None
        self.pose_base = None
        self.yaw_atual = 0.0

        self.timer = self.create_timer(0.1, self.controlar)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.pose_atual = (pos.x, pos.y)
        if self.pose_base is None:
            self.pose_base = (pos.x, pos.y)
            self.get_logger().info(f'Pose inicial registrada: {self.pose_base}')

        # Converte quaternion para yaw
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw_atual = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        indices_frente = list(range(330, 360)) + list(range(0, 31))
        distancias = [msg.ranges[i] for i in indices_frente if not np.isnan(msg.ranges[i])]
        self.obstaculo_frontal = bool(distancias and min(distancias) < self.limite_obstaculo_frontal)

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        lower = np.array([220,  60,   0])
        upper = np.array([255,  90,  30])
        mask = cv2.inRange(frame, lower, upper)
        cv2.imshow("Máscara da Bandeira", mask)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.bandeira_visivel = False
            self.cx = None
            return

        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0:
            self.bandeira_visivel = False
            self.cx = None
            return

        self.cx = int(M['m10'] / M['m00'])
        self.bandeira_visivel = True
        self.bandeira_ja_encontrada = True
        self.get_logger().info(f'bandeira detectada! cx = {self.cx}')

    def controlar(self):
        twist = Twist()
        self.get_logger().info(f'Estado atual: {self.estado.name}')

        if self.estado == EstadoMissao.EXPLORANDO:
            if self.bandeira_visivel:
                self.estado = EstadoMissao.NAVIGANDO_PARA_BANDEIRA
            elif self.obstaculo_frontal:
                twist.linear.x = 0.0; twist.angular.z = 0.5
            else:
                twist.linear.x = 0.2; twist.angular.z = 0.0

        elif self.estado == EstadoMissao.NAVIGANDO_PARA_BANDEIRA:
            if self.bandeira_visivel and self.cx is not None:
                erro = self.cx - 320
                if (90 <= self.cx <= 150) and self.obstaculo_frontal:
                    self.estado = EstadoMissao.POSICIONANDO_PARA_COLETA
                    return
                if self.obstaculo_frontal:
                    twist.linear.x = 0.0; twist.angular.z = 0.5
                elif 90 <= self.cx <= 150:
                    twist.linear.x = 0.15; twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0; twist.angular.z = -0.002 * erro
            else:
                twist.linear.x = 0.0; twist.angular.z = 0.5

        elif self.estado == EstadoMissao.POSICIONANDO_PARA_COLETA:
            if self.bandeira_visivel and self.cx is not None:
                erro = self.cx - 320
                if 110 <= self.cx <= 120:
                    twist.linear.x = 0.0; twist.angular.z = 0.0
                    self.estado = EstadoMissao.CAPTURANDO_BANDEIRA
                else:
                    twist.linear.x = 0.0; twist.angular.z = -0.005 * erro
            else:
                twist.linear.x = 0.0; twist.angular.z = 0.4

        elif self.estado == EstadoMissao.CAPTURANDO_BANDEIRA:
            # Move de [0,0,0] para [0,-0.6,0.6]
            msg = Float64MultiArray(); msg.data = [0.0, -0.6, 0.6]
            self.garra_pub.publish(msg)
            self.get_logger().info('Garra configurada para pegar (0,-0.6,0.6)')
            time.sleep(0.8)

            # Avança para encaixar na bandeira
            twist2 = Twist(); twist2.linear.x = 2.00
            self.cmd_vel_pub.publish(twist2)
            time.sleep(1.8)
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Avançou para encaixar a bandeira')

            # Volta para [0,0,0] fechando a garra
            msg = Float64MultiArray(); msg.data = [0.0, 0.0, 0.0]
            self.garra_pub.publish(msg)
            self.get_logger().info('Garra fechada (0,0,0)')
            time.sleep(0.8)

            # Ajusta haste para data = [-0.5, 0, 0]
            msg = Float64MultiArray(); msg.data = [-0.5, 0.0, 0.0]
            self.garra_pub.publish(msg)
            self.get_logger().info('Haste ajustada para -0.5')
            time.sleep(0.8)

            # Ajusta haste para data = [-0.8, 0, 0]
            msg = Float64MultiArray(); msg.data = [-0.8, 0.0, 0.0]
            self.garra_pub.publish(msg)
            self.get_logger().info('Haste levantada para -0.8')
            time.sleep(0.8)

            self.estado = EstadoMissao.RETORNANDO_BASE
            return

        elif self.estado == EstadoMissao.RETORNANDO_BASE:
            if self.pose_atual is None or self.pose_base is None:
                self.get_logger().warn("Posição atual ou base não definida.")
                return

            x_atual, y_atual = self.pose_atual
            x_base, y_base = self.pose_base
            dx, dy = x_base - x_atual, y_base - y_atual
            distancia = np.hypot(dx, dy)

            angulo_desejado = np.arctan2(dy, dx)
            erro_angular = angulo_desejado - self.yaw_atual
            erro_angular = math.atan2(math.sin(erro_angular), math.cos(erro_angular))

            if abs(erro_angular) > 0.1:
                twist.angular.z = 0.5 * erro_angular; twist.linear.x = 0.0
            elif distancia > 0.4:
                twist.linear.x = 0.15; twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0; twist.angular.z = 0.0
                self.estado = EstadoMissao.POSICIONANDO_PARA_DEPOSITO

        elif self.estado == EstadoMissao.POSICIONANDO_PARA_DEPOSITO:
            twist.linear.x = 0.0; twist.angular.z = 0.0
            self.estado = EstadoMissao.DEPOSITANDO_BANDEIRA

        elif self.estado == EstadoMissao.DEPOSITANDO_BANDEIRA:
            msg = Float64MultiArray(); msg.data = [0.0, 0.0, 0.06]
            self.garra_pub.publish(msg)
            self.get_logger().info('Abaixando e abrindo para depositar')
            time.sleep(0.5)
            self.estado = EstadoMissao.EXPLORANDO

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleMissao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
