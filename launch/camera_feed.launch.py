import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('image_folder', default_value='/home/ana/ros2/arduino_ws',
                          description='Image folder'),
    DeclareLaunchArgument('image_name', default_value='image_test.png',
                          description='Image name'),
    DeclareLaunchArgument('timer_dt', default_value='1.0',
                          description='Time frequency in seconds'),
]


#####################################
def generate_launch_description():

  # Camera
  camera_feed = Node(
          package='ros2_arduino_serial',
          executable='camera_feed',
          name='camera_feed',
          output='screen',
          parameters=[
            {"image_folder": LaunchConfiguration('image_folder')},
            {"image_name": LaunchConfiguration('image_name')},
            {"timer_dt": LaunchConfiguration('timer_dt')} 
          ]
          ) 

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(camera_feed)
  return ld

