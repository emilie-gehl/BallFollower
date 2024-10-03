from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   turtlebot_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch')
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
            PythonLaunchDescriptionSource([turtlebot_launch_file_dir, '/robot.launch.py']),
        )
   ])