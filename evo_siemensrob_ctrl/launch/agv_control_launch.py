import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from CGIHTTPServer import executable
def generate_launch_description():
    ld = LaunchDescription()
    config_dir = LaunchConfiguration('config_dir', 
        default=os.path.join(get_package_share_directory('twist_mux'), 
        'config', 'twist_mux_topics.yaml'))

    joy_node = Node(
        package = 'joy',
        #name = 'joy_node',
        executable = 'joy_node')

    agv_node = Node(
        package = 'evo_siemensrob_ctrl',
        #name = 'evo_siemensrob_ctrl_node',
        executable = 'evo_siemensrob_ctrl_node')

    joytovel_node = Node(
        package ='joytovel',
        #name = 'joytovel',
        executable = 'joytovel')

    joy_converter_node = Node(
        package = 'joy_converter',
        #name = 'joy_converter',
        executable = 'joy_converter')

    twist_mux_node = Node(
      package='twist_mux', 
      executable='twist_mux', 
      output='screen',
      remappings={('/cmd_vel_out', '/twist_mux/cmd_vel')},
      parameters=[config_dir])

    twist_mux_marker = Node(
      package='twist_mux', 
      executable='twist_marker', 
      output='screen',
      remappings={('/twist', '/twist_mux/cmd_vel')},
      parameters=[{
        'frame_id': 'base_link',
        'scale': 1.0,
        'vertical_position': 2.0}])
    
    ld.add_action(joy_node)
    ld.add_action(joy_converter_node)
    ld.add_action(joytovel_node)
    ld.add_action(agv_node)
    ld.add_action(twist_mux_node)
    ld.add_action(twist_mux_marker)

    return ld
