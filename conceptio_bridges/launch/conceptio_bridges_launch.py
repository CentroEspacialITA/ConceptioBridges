import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mqtt_mirror',
            executable='mqtt_mirror',
            name='mqtt_mirror_node'),
  ])