import rclpy
from rclpy.node import Node
from custom_interfaces.msg import BallCoordinates  # Importer le message personnalisé
import cv2 as cv
import numpy as np
import os

os.environ['DISPLAY'] = ':0'

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        
        # Créer un publisher pour publier sur le topic "/ball_coordinates"
        self.publisher_ = self.create_publisher(BallCoordinates, '/ball_coordinates', 10)
        
        # Initialiser la caméra
        self.camera_id = 0
        self.cap = cv.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error("Erreur, impossible d'ouvrir la caméra.")
            return
        
        # Paramètres de la balle
        self.real_diameter_mm = 40  # diamètre de la balle en mm
        self.k = 1400  # Coefficient de calibrage
        
        # Plages de couleur pour l'orange
        self.lower_orange = np.array([0, 50, 150])
        self.upper_orange = np.array([25, 255, 255])


        self.timer_period = 0.14  # Période de publication (toutes les 0.05 secondes)
        self.timer = self.create_timer(self.timer_period, self.detect_and_publish)

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Erreur, impossible de lire le flux vidéo.")
            return

        # Obtenir les dimensions de l'image
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        
        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_frame, self.lower_orange, self.upper_orange)
        blurFrame = cv.GaussianBlur(mask, (17, 17), 0)

        circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1.2, 100,
                                   param1=100, param2=30, minRadius=15, maxRadius=400)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            chosen = circles[0][0]  # Prendre le premier cercle détecté

            if chosen is not None:
                cv.circle(frame, (chosen[0], chosen[1]), 1, (0, 100, 100), 3)
                cv.circle(frame, (chosen[0], chosen[1]), chosen[2], (255, 0, 255), 3)

                diameter_pixels = chosen[2] * 2  # diamètre apparent en pixels
                distance_m = (self.real_diameter_mm * self.k * 0.001) / diameter_pixels  # Calculer la distance

                # Calculer les coordonnées par rapport au centre de l'image
                relative_x = (chosen[0] - center_x) / (width / 2)
                relative_y = -(chosen[1] - center_y) / (height / 2)

                # Publier les coordonnées
                msg = BallCoordinates()
                msg.x = float(relative_x)
                msg.y = float(relative_y)
                msg.distance = distance_m
                self.publisher_.publish(msg)  # Publier le message
                self.get_logger().info(f'Coordonnées publiées : x={msg.x}, y={msg.y}, distance={msg.distance:.2f} m')

                text = f"Coord: ({relative_x:.2f}, {relative_y:.2f}) & Dist: {distance_m:.2f} m"
                cv.putText(frame, text, (chosen[0] + 10, chosen[1] - 10), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        else:   # Aucune balle détectée, publier un message indiquant cela
            msg = BallCoordinates()
            msg.x = 2.0  # Indiquer qu'aucune balle n'est détectée
            msg.y = 2.0  # Indiquer qu'aucune balle n'est détectée
            msg.distance = 0.0  # Optionnel, distance fictive
            self.publisher_.publish(msg)
            self.get_logger().info('Aucune balle détectée. Coordonnées publiées : x=2, y=2')


        cv.imshow('Circles', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv.destroyAllWindows()

    def publish_coordinates(self):
        msg = BallCoordinates()
        msg.x = 0.60  # Coordonnée X fictive
        msg.y = -0.00  # Coordonnée Y fictive
        msg.distance = 0.8  # Distance fictive
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
