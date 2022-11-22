from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='turtlesim',
        #    namespace='turtlesim1',
        #    executable='turtlesim_node',
        #    name='sim'
        #),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.XmlLaunchDescriptionSource(
                get_package_share_directory('rosbridge_server') + '/launch/rosbridge_websocket_launch.xml'
            )
        ),
    ])
