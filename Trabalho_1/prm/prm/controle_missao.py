#importa bibliotecas do ROS 2, mensagens e ferramentas auxiliares
import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2

#enumeração dos estados da missão
class EstadoMissao(Enum):
    EXPLORANDO = 1
    NAVIGANDO_PARA_BANDEIRA = 2
    POSICIONANDO_PARA_COLETA = 3

#classe principal do nó de controle da missão
class ControleMissao(Node):
    def __init__(self):
        super().__init__('controle_missao')

        #estado inicial
        self.estado = EstadoMissao.EXPLORANDO

        #publicador de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #assinantes de sensores
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)
        self.bridge = CvBridge()

        #variáveis sensoriais
        self.obstaculo_frontal = False
        self.bandeira_visivel = False
        self.cx = None
        self.bandeira_ja_encontrada = False

        #timer de controle (10 Hz)
        self.timer = self.create_timer(0.1, self.controlar)

    #processa dados do Lidar
    def scan_callback(self, msg):
        indices_frente = list(range(330, 360)) + list(range(0, 31))
        distancias = [msg.ranges[i] for i in indices_frente if not np.isnan(msg.ranges[i])]
        self.obstaculo_frontal = bool(distancias and min(distancias) < 0.4)

    #processa imagem da câmera e detecta cor da bandeira
    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            lower = np.array([50, 60, 0])
            upper = np.array([65, 80, 20])
            mask = cv2.inRange(frame, lower, upper)

            cv2.imshow("Máscara da Bandeira", mask)
            cv2.waitKey(1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    self.cx = int(M["m10"] / M["m00"])
                    self.bandeira_visivel = True
                    self.bandeira_ja_encontrada = True
                    self.get_logger().info(f'- Bandeira detectada! cx = {self.cx}')
            else:
                self.bandeira_visivel = False
                self.cx = None
        except Exception as e:
            self.get_logger().error(f'Erro no processamento de imagem: {e}')

    #controle principal do comportamento do robô
    def controlar(self):
        twist = Twist()

        if self.estado == EstadoMissao.EXPLORANDO:
            self.get_logger().info('Estado: EXPLORANDO')

            if self.bandeira_visivel:
                self.get_logger().info('- Mudando para NAVIGANDO_PARA_BANDEIRA.')
                self.estado = EstadoMissao.NAVIGANDO_PARA_BANDEIRA
            elif self.obstaculo_frontal:
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0

        elif self.estado == EstadoMissao.NAVIGANDO_PARA_BANDEIRA:
            self.get_logger().info('Estado: NAVIGANDO_PARA_BANDEIRA')

            if self.bandeira_visivel and self.cx is not None:
                erro = self.cx - 320
                self.get_logger().info(f'Navegando: erro de centralização = {erro} (cx = {self.cx})')

                #entra em coleta se bandeira estiver centralizada e próxima
                if self.bandeira_visivel and (90 <= self.cx <= 150) and self.obstaculo_frontal:
                    self.get_logger().info('- Frente da bandeira detectada! Indo para POSICIONANDO_PARA_COLETA.')
                    self.estado = EstadoMissao.POSICIONANDO_PARA_COLETA
                    return

                if self.obstaculo_frontal:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5

                elif 90 <= self.cx <= 150:
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0
                    self.get_logger().info('Bandeira centralizada - andando para frente.')

                else:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.002 * erro
                    self.get_logger().info('Bandeira fora do centro - girando para ajustar.')

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                self.get_logger().warn('Bandeira sumiu da visão! Girando até reencontrar...')

        elif self.estado == EstadoMissao.POSICIONANDO_PARA_COLETA:
            self.get_logger().info('Estado: POSICIONANDO_PARA_COLETA')

            if self.bandeira_visivel and self.cx is not None:
                erro = self.cx - 320

                if 120 <= self.cx <= 150:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info('Bandeira perfeitamente centralizada - parado para coleta.')
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.002 * erro
                    self.get_logger().info('Ajustando orientação fina para coleta.')
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.4
                self.get_logger().warn('Bandeira sumiu! Girando devagar para reencontrar.')

        self.cmd_vel_pub.publish(twist)

#função principal
def main(args=None):
    rclpy.init(args=args)
    node = ControleMissao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
