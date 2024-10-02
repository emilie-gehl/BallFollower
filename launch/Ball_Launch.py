from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   return LaunchDescription([
       # Noeud BallFollower
       Node(
           package='BallFollower',
           executable='Ball_Publisher',
           name='Ball_Publisher',
           output='screen'
       ),
       Node(
           package='BallFollower',
           executable='BallFollower',
           name='BallFollower',
           output='screen'
       ),
       # Inclusion du fichier robot.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('~turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py'),
            # Vous pouvez passer des arguments ici si nécessaire
        )
   ])