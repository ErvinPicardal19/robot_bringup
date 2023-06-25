from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   
   camera_config = os.path.join(get_package_share_directory('robot_bringup'), 'config', 'camera.yaml')
   
   camera_node = Node(     
      package="v4l2_camera",
      executable="v4l2_camera_node",
      output="screen",
      parameters=[camera_config],
   )
      
   # rqt_image_viewer = Node(
   #    package="rqt_image_view",
   #    executable="rqt_image_view",
   #    output="screen"
   # )
   
   return LaunchDescription([
      camera_node,
      # rqt_image_viewer
   ])
   
   
   