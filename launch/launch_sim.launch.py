import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
   use_ros2_control = LaunchConfiguration('use_ros2_control')
   use_sim_time = LaunchConfiguration('use_sim_time')
   
   
   pkg_directory = get_package_share_directory('robot_bringup')
   gazebo_directory = get_package_share_directory('gazebo_ros')
   
   rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         pkg_directory, 'launch', 'rsp.launch.py'
      )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
   )
   
   gazebo_params_file = os.path.join(pkg_directory,'config','gazebo_params.yaml')
   
   gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
   
   spawn_entity_node = Node(
      package="gazebo_ros",
      executable="spawn_entity.py",
      arguments=['-topic', 'robot_description',
                 '-entity', 'robot'],
      output='screen'
   )
   
   diff_cont_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=['diff_cont']
   )
   
   joint_broad_controller = Node(
      package="controller_manager",
      executable="spawner",
      arguments=['joint_broad']
   )
   
   return LaunchDescription([
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_broad_controller]
         )
      ),
      
      RegisterEventHandler(
         event_handler=OnProcessExit(
            target_action=joint_broad_controller,
            on_exit=[diff_cont_controller]
         )
      ),
      
      # RegisterEventHandler(
      #    event_handler=OnProcessExit(
      #       target_action=diff_cont_controller,
      #       on_exit=[]
      #    )
      # ),
      
      
      rsp,
      gazebo,
      spawn_entity_node,
      
   ])