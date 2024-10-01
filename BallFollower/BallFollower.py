import rclpy
from rclpy.node import Node
from custom_interfaces.msg import BallCoordinates  # Assure-toi que le chemin est correct
from geometry_msgs.msg import Twist


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
        self.timer = self.create_timer(0.1, self.move_towards_ball)
        

    def listener_callback(self, msg):
        # Mettre à jour les données reçues
        self.x = msg.x
        self.y = msg.y
        self.distance = msg.distance
        
        # Affiche ou traite les coordonnées reçues
        self.get_logger().info(f'Balle détectée à x={self.x}, y={self.y}, distance={self.distance}')
    
    def move_towards_ball(self):
        # Calculer et commander les moteurs du TurtleBot en fonction des dernières coordonnées reçues
        msg = Twist()
        msg.linear.x = min(self.distance * 0.22, 0.22)  # Limiter la vitesse linéaire
        msg.angular.z = -self.x * 2.84  # Ajuster la direction selon la position de la balle
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    ball_follower = BallFollower()
    
    rclpy.spin(ball_follower)  # Attendre et écouter les messages
    
    ball_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

