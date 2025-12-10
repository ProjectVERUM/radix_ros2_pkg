import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = "radix_ros"
BEHAVIOR = "basic"

params_dir = Path(get_package_share_directory(PACKAGE)) / "params" / BEHAVIOR
main_params = str(params_dir / "main_params.yaml")
label_params = str(params_dir / "label_params.yaml")
rules_params = str(params_dir / "rules.yaml")


def generate_launch_description():
    node = Node(
        package=PACKAGE,
        executable="radix_server_node",
        name="radix_server",
        arguments=["--ros-args", "--log-level", os.getenv("LOG", "INFO")],
        prefix="gdbserver localhost:3000 --" if os.getenv("DEBUG", "0") == "1" else "",
        parameters=[
            main_params,
            {"label_params_path": label_params},
            {"rules_path": rules_params},
        ],
    )
    return LaunchDescription([node])
