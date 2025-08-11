import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os

def generate_triangle_waypoints(base, height, z):
    A = {"x": -base / 2, "y": 0, "z": z, "yaw": 0}
    B = {"x": base / 2, "y": 0, "z": z, "yaw": 0}
    C = {"x": 0, "y": height, "z": z, "yaw": 0}
    return {
        "/px4_1": [A],  # فقط نقطه A برای پرنده ۱
        "/px4_2": [B],  # فقط نقطه B برای پرنده ۲
        "/px4_3": [C]   # فقط نقطه C برای پرنده ۳
    }

def generate_line_waypoints(length, z):
    A = {"x": -length / 2, "y": 0, "z": z, "yaw": 0}  # نقطه شروع
    M = {"x": 0, "y": 0, "z": z, "yaw": 0}            # نقطه وسط
    B = {"x": length / 2, "y": 0, "z": z, "yaw": 0}   # نقطه پایان
    return {
        "/px4_1": [A],  # پرنده ۱ تو نقطه شروع
        "/px4_2": [M],  # پرنده ۲ تو وسط
        "/px4_3": [B]   # پرنده ۳ تو نقطه پایان
    }

def generate_quadrilateral_waypoints(width, height, z):
    A = {"x": -width/2, "y": -height/2, "z": z, "yaw": 0}
    B = {"x": width/2, "y": -height/2, "z": z, "yaw": 0}
    C = {"x": width/2, "y": height/2, "z": z, "yaw": 0}
    D = {"x": -width/2, "y": height/2, "z": z, "yaw": 0}
    return {
        "/px4_1": [A],  # رأس A
        "/px4_2": [B],  # رأس B
        "/px4_3": [C],  # رأس C
        "/px4_4": [D]   # رأس D
    }

def generate_swarm(num_drones):
    swarm = {}
    spacing = 2.0  # فاصله بین کوادکوپترها
    for i in range(1, num_drones + 1):
        x = (i - 2) * spacing
        y = 10.0
        swarm[str(i)] = {
            "model": "iris",
            "initial_pose": {"x": x, "y": y},
            "is_leader": True
        }
    return swarm

def parse_swarm_config(swarm_config):
    model_counts = {}
    for key, item in swarm_config.items():
        model = item["model"]
        model_counts[model] = model_counts.get(model, 0) + 1

    script = ",".join([f"{key}:{item}" for key, item in model_counts.items()])

    initial_poses_dict = {}
    initial_poses_string = "\""
    is_leaders = []
    for idx, item in enumerate(swarm_config.values()):
        initial_pose = item["initial_pose"]
        initial_poses_dict[f"px4_{idx + 1}"] = initial_pose
        initial_poses_string += f"{initial_pose['x']},{initial_pose['y']}|"
        is_leaders.append(item["is_leader"])
    initial_poses_string = initial_poses_string[:-1] + "\""

    return len(swarm_config), script, initial_poses_string, initial_poses_dict, is_leaders

def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('px4_swarm_controller')

    # حذف فایل waypoints_translated.yaml برای جلوگیری از استفاده ناخواسته
    ld.add_action(ExecuteProcess(cmd=['rm', '-f', os.path.join(package_dir, 'config', 'Trajectories', 'waypoints_translated.yaml')]))

    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as swarm_file:
        swarm_config_data = json.load(swarm_file)

    if "active_shape" in swarm_config_data and "formations" in swarm_config_data:
        shape = swarm_config_data["active_shape"]
        form = swarm_config_data["formations"][shape]
        z = form.get("flight_altitude", -10.0)
        num_drones = form.get("num_drones", 3)
        swarm = generate_swarm(num_drones)
    else:
        raise ValueError("Missing 'active_shape' or 'formations' in config")

    if shape == "triangle":
        base = form["base"]
        height = form["height"]
        waypoints = generate_triangle_waypoints(base, height, z)
    elif shape == "line":
        length = form["length"]
        waypoints = generate_line_waypoints(length, z)
    elif shape == "quadrilateral":
        width = form["width"]
        height = form["height"]
        waypoints = generate_quadrilateral_waypoints(width, height, z)
    else:
        raise ValueError(f"Unknown shape type: {shape}")

    wp_path = os.path.join(package_dir, "config", "Trajectories", "waypoints_auto.yaml")

    with open(wp_path, "w") as f:
        f.write("threshold: 0.1\n")
        f.write("threshold_angle: 0.05\n")
        f.write("wp:\n")
        for drone_ns, points in waypoints.items():
            f.write(f"  {drone_ns}:\n")
            for pt in points:
                f.write(f"    - {{ x: {pt['x']}, y: {pt['y']}, z: {pt['z']}, yaw: {pt['yaw']} }}\n")

    swarm_config_data["swarm"] = swarm
    nb_drones, script, initial_poses, initial_poses_dict, is_leaders = parse_swarm_config(swarm)

    with open(os.path.join(package_dir, 'config', 'control_config.json'), 'r') as control_file:
        control_config = json.load(control_file)

    neighborhood = control_config["neighborhood"]
    neighbors_exe = neighborhood["neighbors_exe"]
    neighbors_distance = neighborhood["neighbor_distance"]
    neighbors_params = neighborhood["params"]

    controller_info = control_config["controller"]
    controller_exe = controller_info["controller_exe"]
    controller_params = controller_info["params"]
    is_leader_follower_control = controller_info["leader_follower"]

    # اطمینان از اینکه leaders همیشه پاس داده می‌شود
    neighbors_params = {"leaders": is_leaders, **neighbors_params}

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='simulation_node.py',
            name='simulation_node',
            parameters=[{'script': script, 'initial_pose': initial_poses}]
        )
    )

    xs_init = []
    ys_init = []

    for (namespace, initial_pose), aleader in zip(initial_poses_dict.items(), is_leaders):
        xs_init.append(initial_pose["y"])
        ys_init.append(initial_pose["x"])

        if aleader:
            ld.add_action(
                Node(
                    package='px4_swarm_controller',
                    executable='waypoint',
                    name='waypoint',
                    namespace=namespace,
                    parameters=[
                        {
                            "wp_path": wp_path,
                            "x_init": initial_pose["x"],
                            "y_init": initial_pose["y"],
                            "enable_translation": False  # غیرفعال کردن انتقال
                        }
                    ]
                )
            )
        else:
            ld.add_action(
                Node(
                    package='px4_swarm_controller',
                    executable=controller_exe,
                    name='controller',
                    namespace=namespace,
                    parameters=[controller_params]
                )
            )

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable=neighbors_exe,
            name='neighbors',
            parameters=[
                {
                    "nb_drones": nb_drones,
                    "neighbor_distance": neighbors_distance,
                    "x_init": xs_init,
                    "y_init": ys_init,
                    "leaders": is_leaders,  # اضافه کردن صریح leaders
                    **neighbors_params
                }
            ]
        )
    )

    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='arming',
            name='arming',
            namespace='simulation',
            parameters=[{"nb_drones": nb_drones}]
        )
    )

    return ld