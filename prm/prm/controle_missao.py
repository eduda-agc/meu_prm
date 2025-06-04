import rclpy
from rclpy.node import Node
from enum import Enum
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2

class EstadoMissao(Enum):
    EXPLORANDO = 1
    BANDEIRA_DETECTADA = 2
    NAVIGANDO_PARA_BANDEIRA = 3
    POSICIONANDO_PARA_COLETA = 4

class ControleMissao(Node):
    def __init__(self):
        super().__init__('controle_missao')
        self.estado = EstadoMissao.EXPLORANDO

        # Publicador de velocidades
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Assinantes de sensores
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilitário para converter imagem
        self.bridge = CvBridge()

        # Informacoes sensoriais
        self.obstaculo_a_frente = False
        self.bandeira_visivel = False
        self.frames_sem_bandeira = 0  # contador de frames consecutivos sem bandeira
        self.cx = None  # centro do blob na imagem

        # Timer principal da missão (10 Hz)
        self.timer = self.create_timer(0.1, self.controlar)

    def scan_callback(self, msg):
        # Verifica obstáculo a frente (-30 a +30 graus)
        indices_frente = list(range(330, 360)) + list(range(0, 31))
        distancias = [msg.ranges[i] for i in indices_frente if not np.isnan(msg.ranges[i])]
        if distancias and min(distancias) < 0.5:
            self.obstaculo_a_frente = True
        else:
            self.obstaculo_a_frente = False

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Cor aproximada da bandeira (ajustada para seu caso)
            lower = np.array([50, 60, 0])
            upper = np.array([65, 80, 20])
            mask = cv2.inRange(frame, lower, upper)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    self.cx = int(M["m10"] / M["m00"])
                    self.bandeira_visivel = True
                    self.frames_sem_bandeira = 0  # reset ao ver a bandeira
                    if self.estado == EstadoMissao.EXPLORANDO:
                        self.estado = EstadoMissao.BANDEIRA_DETECTADA
            else:
                self.frames_sem_bandeira += 1
                if self.frames_sem_bandeira > 5:
                    self.bandeira_visivel = False
        except Exception as e:
            self.get_logger().error(f'Erro no processamento de imagem: {e}')

        cv2.imshow("Máscara", mask)
        cv2.waitKey(1)



    def controlar(self):
        twist = Twist()

        if self.estado == EstadoMissao.EXPLORANDO:
            self.get_logger().info('Estado: EXPLORANDO')
            if self.obstaculo_a_frente:
                twist.angular.z = 0.4
            else:
                twist.linear.x = 0.2

        elif self.estado == EstadoMissao.BANDEIRA_DETECTADA:
            self.get_logger().info('Estado: BANDEIRA_DETECTADA')
            if self.bandeira_visivel:
                img_centro = 320 
                erro = self.cx - img_centro
                if abs(erro) > 30:
                    twist.angular.z = -0.002 * erro
                else:
                    self.estado = EstadoMissao.NAVIGANDO_PARA_BANDEIRA
            elif self.frames_sem_bandeira > 10:
                self.estado = EstadoMissao.EXPLORANDO


        elif self.estado == EstadoMissao.NAVIGANDO_PARA_BANDEIRA:
            self.get_logger().info('Estado: NAVIGANDO_PARA_BANDEIRA')
            img_centro = 320

            if self.bandeira_visivel:
                erro = self.cx - img_centro
                if abs(erro) > 30:
                    twist.angular.z = -0.002 * erro
                    twist.linear.x = 0.05  # anda e corrige orientação ao mesmo tempo
                else:
                    twist.linear.x = 0.15  # alinhado, pode ir em frente
                self.frames_sem_bandeira = 0  # reseta contador
            else:
                self.frames_sem_bandeira += 1
                if self.frames_sem_bandeira <= 10:
                    twist.angular.z = 0.3  # tenta reencontrar a bandeira girando
                else:
                    self.estado = EstadoMissao.EXPLORANDO


        elif self.estado == EstadoMissao.POSICIONANDO_PARA_COLETA:
            self.get_logger().info('Estado: POSICIONANDO_PARA_COLETA')
            # Lógica extra pode ser adicionada aqui
            twist.linear.x = 0.05

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControleMissao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
