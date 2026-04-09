import os
import random
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
import xacro

# ==============================================================================
# MACRO EQUIVALENT: Hardcoded Fleet Size Constraint
# Modify this value directly to scale the swarm. Do not use Launch Configurations.
# ==============================================================================
FLEET_SIZE = 4

# Farm limits based on visual buoys (Leaves a 5m buffer from the edge)
SPAWN_MIN = -45.0
SPAWN_MAX = 45.0
MIN_SPAWN_DISTANCE = 3.0  # Minimum meters between spawned ASVs to prevent Gazebo explosions

def generate_random_safe_coordinates(existing_coords):
    """
    Generates a random (X, Y) coordinate that is guaranteed to be at least 
    MIN_SPAWN_DISTANCE away from all previously spawned agents.
    """
    while True:
        x = random.uniform(SPAWN_MIN, SPAWN_MAX)
        y = random.uniform(SPAWN_MIN, SPAWN_MAX)
        
        collision = False
        for (ex, ey) in existing_coords:
            dist = math.sqrt((x - ex)**2 + (y - ey)**2)
            if dist < MIN_SPAWN_DISTANCE:
                collision = True
                break
        
        if not collision:
            return x, y

def spawn_asv_swarm(context, *args, **kwargs):
    # Retrieve the path to the URDF
    pkg_share = get_package_share_directory('matsyota_simulator')
    xacro_file = os.path.join(pkg_share, 'urdf', 'asv.urdf.xacro')

    spawn_actions = []
    spawned_coordinates = []

    for i in range(FLEET_SIZE):
        namespace = f'asv_{i}'
        
        # 1. Compile Xacro with the specific namespace argument
        doc = xacro.process_file(xacro_file, mappings={'namespace': namespace})
        robot_description = {'robot_description': doc.toxml()}

        # 2. Get Safe Random Coordinates
        spawn_x, spawn_y = generate_random_safe_coordinates(spawned_coordinates)
        spawned_coordinates.append((spawn_x, spawn_y))

        # 3. Robot State Publisher (Isolated via namespace)
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        )

        # 4. Spawn Entity in Gazebo
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{namespace}/robot_description',
                '-entity', namespace,
                '-x', str(spawn_x),
                '-y', str(spawn_y),
                '-z', '0.0'
            ],
            output='screen'
        )

        spawn_actions.extend([rsp_node, spawn_node])

    return spawn_actions

def generate_launch_description():
    # Start Gazebo with the aquatic farm world
    pkg_share = get_package_share_directory('matsyota_simulator')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_water.world')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        OpaqueFunction(function=spawn_asv_swarm)
    ])