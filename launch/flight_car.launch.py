
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    twist_odom_convert_launch = PathJoinSubstitution(
        [get_package_share_directory("rover_demo"), 'launch', 'flight_twist_odom_convert.launch.py'])

        
    twist_odom_convert = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([twist_odom_convert_launch]),
        launch_arguments = {
          'odom_in': '/odom',
          'twist_out': '/cmd_vel'          
        }.items(),        
    )

  # Node uses https://github.com/ana-GT/ros2_arduino_serial to send data to car
  car_node = Node(
          package='ros2_arduino_serial',
          executable='car_comm',
          name='car_comm',
          output='screen',
  ) 

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(twist_odom_convert)
    ld.add_action(car_node)
    return ld
