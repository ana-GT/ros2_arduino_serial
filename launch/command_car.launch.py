import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro

#####################################
def generate_launch_description():

  # rqt node
  rqt_node = ExecuteProcess(
    cmd = ['rqt', '--perspective-file', os.path.join(get_package_share_directory("ros2_arduino_serial"), "config", "steering.perspective")],
    shell = True
    )
  
  # Car
  control_car = Node(
          package='ros2_arduino_serial',
          executable='car_comm',
          name='car_comm',
          output='screen' #,
          #parameters=[
          #  {"joint_names": ["Base_Joint", "Shoulder_Roll", "Shoulder_Yaw", "Elbow_Pitch", "Wrist_Pitch", "Wrist_Yaw", "Wrist_Roll"]} 
          #]
          ) 

  return LaunchDescription(
      [
       rqt_node,
       control_car
      ]
  )
