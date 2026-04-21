import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_simulator = get_package_share_directory('matsyota_simulator')
    # Use xacro if it's a xacro file
    urdf_file = os.path.join(pkg_simulator, 'urdf', 'asv.urdf.xacro')
    world_file = os.path.join(pkg_simulator, 'worlds', 'empty_water.world')
    
    # Run xacro to generate the robot description
    import xacro
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toprettyxml(indent='  ')

    ld = LaunchDescription()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    ld.add_action(gazebo)

    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    ld.add_action(rviz_node)

    # Broadcast static transform from 'map' to 'odom'
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    ld.add_action(static_tf_node)

    # Global Nodes: commander and mapper
    commander_node = Node(
        package='matsyota_controller', 
        executable='matsyota_commander',
        name='matsyota_commander',
        output='screen'
    )
    ld.add_action(commander_node)

    mapper_node = Node(
        package='matsyota_controller',
        executable='feed_dispersion_mapper',
        name='feed_dispersion_mapper',
        output='screen'
    )
    ld.add_action(mapper_node)

    FLEET_SIZE = 4
    for i in range(FLEET_SIZE):
        namespace = f'asv_{i}'
        
        # Determine collision-safe coordinates (grid layout)
        x_pos = float((i % 2) * 3.0) - 1.5
        y_pos = float((i // 2) * 3.0) - 1.5

        # Robot State Publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )
        ld.add_action(rsp_node)

        # Spawn in Gazebo
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_entity_{i}',
            namespace=namespace,
            arguments=[
                '-entity', namespace,
                '-topic', 'robot_description',
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', '0.5'
            ],
            output='screen'
        )
        ld.add_action(spawn_node)

        # ASV Driver
        driver_node = Node(
            package='matsyota_agent',  # Assuming it's in this package
            executable='asv_driver',
            name='asv_driver',
            namespace=namespace,
            output='screen',
            parameters=[{'namespace': namespace}]
        )
        ld.add_action(driver_node)

    return ld
