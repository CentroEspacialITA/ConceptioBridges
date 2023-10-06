import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    tcp_port = launch.substitutions.LaunchConfiguration('tcp_port')
    websocket_port = launch.substitutions.LaunchConfiguration('websocket_port')
    mqtt_host = launch.substitutions.LaunchConfiguration('mqtt_host')
    mqtt_port = launch.substitutions.LaunchConfiguration('mqtt_port')

    tcp_port_launch_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='9091',
        description='TCP port for rosbridge server')

    websocket_port_launch_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='9090',
        description='Websocket port for rosbridge server')

    mqtt_host_launch_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='localhost',
        description='MQTT host for mqtt_mirror')

    mqtt_port_launch_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT port for mqtt_mirror')

    rosbridge_websocket_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch'), '/rosbridge_websocket_launch.xml']),
            # Use the default port from DeclareLaunchArgument
            launch_arguments= {'port': websocket_port}.items(),
        )

    rosbridge_tcp_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch'), '/rosbridge_tcp_launch.xml']),
            launch_arguments= {'port': tcp_port, 'bson_only_mode': 'true'}.items(),
         )

    mqtt_mirror_node = launch_ros.actions.Node(
            package='mqtt_mirror',
            executable='mqtt_mirror',
            parameters=[{'mqtt_host': mqtt_host, 'mqtt_port': mqtt_port}],
            name='mqtt_mirror_node')
    
    return launch.LaunchDescription([
        tcp_port_launch_arg,
        websocket_port_launch_arg,
        mqtt_host_launch_arg,
        mqtt_port_launch_arg,
        rosbridge_websocket_launch,
        rosbridge_tcp_launch,
        mqtt_mirror_node
    ])