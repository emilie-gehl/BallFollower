import rclpy
from rclpy.node import Node
from custom_interfaces.msg import BallCoordinates  # Assure-toi que le chemin est correct
from geometry_msgs.msg import Twist
import time


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
        # S'abonner au topic "/ball_coordinates"
        self.subscription = self.create_subscription(
            BallCoordinates,  # Type de message que ta camarade envoie
            '/ball_coordinates',  # Le topic auquel on s'abonne
            self.listener_callback,  # Callback quand on reçoit un message
            10)
        self.subscription  # Empêche la destruction immédiate du subscriber
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Variables pour stocker les données reçues
        self.x = 0.0
        self.y = 0.0
        self.distance = 0.0

        # Créer un timer pour mettre à jour la commande de mouvement régulièrement
        self.timer = self.create_timer(0.20, self.move_towards_ball)

    def listener_callback(self, msg):
        # Mettre à jour les données reçues
        self.x = msg.x
        self.y = msg.y
        self.distance = msg.distance
    
    def move_towards_ball(self):
        # Calculer et commander les moteurs du TurtleBot en fonction des dernières coordonnées reçues
        msg = Twist()

        # Vérifier si aucune balle n'est détectée
        if self.x == 2 and self.y == 2:
            # Tourner lentement dans un sens pour retrouver la balle
            msg.angular.z = 0.5  # Ajuste cette valeur pour contrôler la vitesse de rotation
            msg.linear.x = 0.0  # Ne pas avancer
            self.get_logger().info('Aucune balle détectée, rotation en cours...')
  #      elif self.distance < 0.3 and self.x <0.75 and self.x>-0.75:
            #Avancer tout droit quand la balle est proche pour shooter la balle
  #          msg.angular.z = 0.0
  #          msg.linear.x = 0.22  
  #          self.get_logger().info('Tentative de buuuuuuuut!!!')
  #          time.sleep(2)
        else:
            # Calculer et commander les moteurs du TurtleBot en fonction des coordonnées reçues
            msg.linear.x = max(0.15, min(self.distance * 0.44, 0.22))
            #msg.linear.x = 0.22
            msg.angular.z = -self.x * 0.5  # Ajuster la direction selon la position de la balle
            # Affiche ou traite les coordonnées reçues
            self.get_logger().info(f'Balle détectée à x={self.x}, y={self.y}, distance={self.distance}')
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    ball_follower = BallFollower()
    
    rclpy.spin(ball_follower)  # Attendre et écouter les messages
    
    ball_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

