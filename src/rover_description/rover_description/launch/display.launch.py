import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'rover_description'
    
    # Dokładne ścieżki na podstawie Twojego zrzutu ekranu
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'Rover.xacro')
    rviz_config = os.path.join(get_package_share_directory(pkg_name), 'launch', 'urdf.rviz')
    
    # Użycie pakietu xacro do konwersji pliku w locie
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    return LaunchDescription([
        # Node publikujący pozycje kół (suwaki)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Node publikujący transformacje całego robota
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        # Node RViz2 do wizualizacji z Twoim plikiem konfiguracyjnym
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[]
        )
    ])