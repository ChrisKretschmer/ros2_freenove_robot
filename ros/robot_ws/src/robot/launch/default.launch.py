import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('robot'),
        'config',
        'config.yaml'
        )
        
    cameraNode=Node(
        package = 'v4l2_camera',
        name = 'camera_node',
        executable = 'v4l2_camera_node',
        parameters = [config]
    )
    
    ultrasonicNode=Node(
        package = 'robot',
        name = 'ultrasonic_node',
        executable = 'ultrasonic',
        parameters = [config]
    )
    
    batteryNode=Node(
        package = 'robot',
        name = 'battery_node',
        executable = 'battery',
        parameters = [config]
    )
        
    motorNode=Node(
        package = 'robot',
        name = 'motor_node',
        executable = 'motor',
        parameters = [config]
    )
    
    baselinkToCameralink=Node(
        package = 'tf2_ros',
        name = 'baselink_to_cameralink',
        executable = 'static_transform_publisher',
        arguments = ["1.2", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "camera_link"]
    )
    
    baselinkToRPICameralink=Node(
        package = 'tf2_ros',
        name = 'baselink_to_cameralink',
        executable = 'static_transform_publisher',
        arguments = ["1.2", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "camera"]
    )
    
    odomToBaselink=Node(
        package = 'tf2_ros',
        name = 'odom_to_baselink',
        executable = 'static_transform_publisher',
        arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "base_link"]
    )
    
    mapToOdom=Node(
        package = 'tf2_ros',
        name = 'map_to_odom',
        executable = 'static_transform_publisher',
        arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"]
    )
    
    cameraLd=IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
    )
    
    rosbridgeServerLd=IncludeLaunchDescription(
      XMLLaunchDescriptionSource([get_package_share_directory('rosbridge_server'), '/launch/rosbridge_websocket_launch.xml']),
    )
    
    ld.add_action(cameraLd)
    ld.add_action(rosbridgeServerLd)

    ld.add_action(baselinkToRPICameralink)
    ld.add_action(baselinkToCameralink)
    ld.add_action(odomToBaselink)
    ld.add_action(mapToOdom)
    
    ld.add_action(cameraNode)
    ld.add_action(ultrasonicNode)
    ld.add_action(batteryNode)
    ld.add_action(motorNode)
    return ld