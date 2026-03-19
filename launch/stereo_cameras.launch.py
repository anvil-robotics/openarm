"""Launch two RealSense D405 cameras (left + right) streaming stereo IR."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rs_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
    ])

    common_params = {
        "enable_color": "false",
        "enable_depth": "false",
        "enable_infra1": "true",
        "enable_infra2": "true",
        "depth_module.profile": "'1280x720x15'",
        "infra_qos": "SENSOR_DATA",
        "infra_info_qos": "SENSOR_DATA",
    }

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "camera_name": "camera_left",
            "camera_namespace": "",
            "serial_no": "'323622273010'",
            **common_params,
        }.items(),
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "camera_name": "camera_right",
            "camera_namespace": "",
            "serial_no": "'230322273305'",
            **common_params,
        }.items(),
    )

    return LaunchDescription([left_camera, right_camera])
