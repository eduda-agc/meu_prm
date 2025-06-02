import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
from math import sqrt

from meu_prm.planejamento import Mapa2D, a_estrela

class ControleMissao(Node):
    def __init__(self):
        super().__init__('controle_missao')

        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.odom_callback, 10)
        self.detect_sub = self.create_subscription(Bool, 'bandeira_detectada', self.detect_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.dist_frente = float('inf')
        self.dist_direita = float('inf')

        self.estado = 'EXPLORANDO'
        self.posicao_atual = (0, 0)
        self.orientacao_atual = 0.0
        self.posicao_anterior = (0, 0)
        self.contador_parado = 0
        self.tempo_desviando = 0

        self.mapa = Mapa2D(10, 10)
        self.caminho = []
        self.indice_caminho = 0
        self.bandeira_detectada = False
        self.posicao_bandeira = (7, 7)  # pode ajustar

        self.timer = self.create_timer(0.1, self.update)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.posicao_atual = (int(x), int(y))

        q = msg.pose.pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        self.orientacao_atual = yaw

    def lidar_callback(self, msg):
        ranges = msg.ranges
        self.dist_frente = min(min(ranges[0:10] + ranges[-10:]), 10.0)
        self.dist_direita = min(ranges[270:290]) if len(ranges) > 290 else float('inf')

        if self.dist_frente < 0.4:
            self.estado = 'DESVIANDO'

    def detect_callback(self, msg):
        if msg.data:
            self.get_logger().info('Bandeira detectada!')
            self.bandeira_detectada = True
            self.estado = 'BANDEIRA_DETECTADA'

    def update(self):
        self.get_logger().info(f'Estado atual: {self.estado}')

        if self.estado == 'EXPLORANDO':
            self.explorar()

        elif self.estado == 'BANDEIRA_DETECTADA':
            self.get_logger().info('Planejando rota até a bandeira...')
            self.caminho = a_estrela(self.mapa, self.posicao_atual, self.posicao_bandeira)
            self.indice_caminho = 0
            self.estado = 'NAVIGANDO'

        elif self.estado == 'NAVIGANDO':
            self.navegar_para_bandeira()

        elif self.estado == 'DESVIANDO':
            self.desviar()

    def explorar(self):
        twist = Twist()

        # de cara na parede, recuar e girar
        if self.dist_frente < 0.3:
            twist.linear.x = -0.1  # recua
            # Gira para o lado com mais espaço
            if self.dist_direita > 0.5:
                twist.angular.z = -0.5  #direita
            else:
                twist.angular.z = 0.5   # esquerda
            self.cmd_vel_pub.publish(twist)
            return

        # livre, segue reto com correções
        twist.linear.x = 0.25
        if self.dist_direita > 0.6:
            twist.angular.z = -0.2  # se afasta da parede
        elif self.dist_direita < 0.3:
            twist.angular.z = 0.2   # perto demais

        self.cmd_vel_pub.publish(twist)


    def desviar(self):
        self.tempo_desviando += 1
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

        if self.tempo_desviando > 20 or (self.dist_frente > 0.4 and self.dist_direita < 1.0):
            self.tempo_desviando = 0
            self.estado = 'EXPLORANDO'

    def navegar_para_bandeira(self):
        if self.indice_caminho >= len(self.caminho):
            self.estado = 'POSICIONANDO'
            return

        destino = self.caminho[self.indice_caminho]
        dx = destino[0] - self.posicao_atual[0]
        dy = destino[1] - self.posicao_atual[1]
        dist = sqrt(dx**2 + dy**2)

        if dist < 0.5:
            self.indice_caminho += 1
        else:
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)

        if self.posicao_atual == self.posicao_anterior:
            self.contador_parado += 1
        else:
            self.contador_parado = 0

        self.posicao_anterior = self.posicao_atual

        if self.contador_parado > 10:
            self.estado = 'DESVIANDO'

def main(args=None):
    rclpy.init(args=args)
    node = ControleMissao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
