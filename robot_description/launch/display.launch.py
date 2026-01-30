
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

rvizRelativePath='config/config.rviz'

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    urdfModelPath = os.path.join(pkgPath, 'urdf/robot_arm.urdf')
    ros2controllerPath = os.path.join(pkgPath, 'config/robot_arm_controller.yaml')
    rvizConfigPath = os.path.join(pkgPath,rvizRelativePath)
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output ='screen',
        parameters=[params, {'use_sim_time': False}, {'ignore_timestamp': True}]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name = 'rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )


    return LaunchDescription([
        DeclareLaunchArgument(name= 'model', default_value = urdfModelPath,
                              description='Path to the urdf model file'),

        robot_state_publisher_node,
        rviz_node,
        joy_node,



    ])