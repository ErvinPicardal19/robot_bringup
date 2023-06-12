import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
   use_sim_time = LaunchConfiguration('use_sim_time')
   
   pkg_directory = get_package_share_directory('robot_bringup')
   gazebo_directory = get_package_share_directory('gazebo_ros')
   
   rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_directory, 'launch', 'rsp.launch.py'
      )]), launch_arguments={'use_sim_time': 'true'}.items()
   )
   
   gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         gazebo_directory, 'launch', 'gazebo.launch.py'
      )]), launch_arguments={'use_sim_time': 'true'}.items()
   )
   
   spawn_entity_node = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      arguments=['-topic', 'robot_description',
                 '-entity', 'robot'],
      output='screen'
   )
   
   return LaunchDescription([
      rsp,
      gazebo,
      spawn_entity_node
   ])