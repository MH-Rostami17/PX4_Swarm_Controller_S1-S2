from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drones = ['px4_1', 'px4_2', 'px4_3', 'px4_4']  # هر تعداد که نیاز داری
    dx, dy, dz, yaw = 0.0, 0.0, 0.0, 0.0  # تغییر بده به نیازت

    return LaunchDescription([
        Node(
            package='px4_swarm_controller',
            executable='shape_mover_node.py',
            name='shape_mover_' + ns,
            namespace='/' + ns,
            parameters=[
                {'namespace': '/' + ns},
                {'input_wp_path': '/home/mhrostami/ros2_ws/install/px4_swarm_controller/share/px4_swarm_controller/config/Trajectories/waypoints_auto.yaml'},
                {'dx': dx},
                {'dy': dy},
                {'dz': dz},
                {'yaw': yaw},
                {'duration': 6.0}
            ]
        ) for ns in drones
    ])