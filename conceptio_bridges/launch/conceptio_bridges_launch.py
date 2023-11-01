import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    mqtt_host = launch.substitutions.LaunchConfiguration('mqtt_host')
    mqtt_port = launch.substitutions.LaunchConfiguration('mqtt_port')

    mqtt_host_launch_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='emqx',
        description='MQTT host for mqtt_mirror')

    mqtt_port_launch_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT port for mqtt_mirror')

    mqtt_mirror_node = launch_ros.actions.Node(
            package='mqtt_mirror',
            executable='mqtt_mirror',
            parameters=[{'mqtt_host': mqtt_host, 'mqtt_port': mqtt_port}],
            name='mqtt_mirror_node')
    
    return launch.LaunchDescription([
        mqtt_host_launch_arg,
        mqtt_port_launch_arg,
        mqtt_mirror_node
    ])