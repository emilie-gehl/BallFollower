import rclpy
from rclpy.node import Node
from custom_interfaces.msg import BallCoordinates  # Importer le message personnalisé

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        
        # Créer un publisher pour publier sur le topic "/ball_coordinates"
        self.publisher_ = self.create_publisher(BallCoordinates, '/ball_coordinates', 10)
        timer_period = 0.5  # Période de publication (toutes les 0.5 secondes)
        self.timer = self.create_timer(timer_period, self.publish_coordinates)

    def publish_coordinates(self):
        msg = BallCoordinates()
        msg.x = 0.30  # Coordonnée X fictive
        msg.y = -0.05  # Coordonnée Y fictive
        msg.distance = 1.2  # Distance fictive
        self.publisher_.publish(msg)  # Publier le message
        self.get_logger().info(f'Coordonnées publiées : x={msg.x}, y={msg.y}, distance={msg.distance}')

def main(args=None):
    rclpy.init(args=args)
    ball_publisher = BallPublisher()
    rclpy.spin(ball_publisher)
    ball_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

