"""Launch two RealSense D405 cameras (stereo IR) and a USB visual-servo camera."""

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
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

    visual_servo_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_visual_servo",
        parameters=[{
            "video_device": "/dev/camera_visual_servo",
            "pixel_format": "YUYV",
            "image_size": [640, 480],
            "time_per_frame": [1, 30],
            "camera_frame_id": "camera_visual_servo",
            "focus_automatic_continuous": False,
            "focus_absolute": 300,
            "white_balance_automatic": False,
            "white_balance_temperature": 4600,
            "auto_exposure": 1,  # Manual Mode
            "exposure_time_absolute": 350,
        }],
        remappings=[
            ("image_raw", "/camera_visual_servo/image_raw"),
            ("camera_info", "/camera_visual_servo/camera_info"),
        ],
    )

    # set_manual_focus = TimerAction(
    #     period=2.0,
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 "v4l2-ctl", "-d", "/dev/camera_visual_servo",
    #                 "-c", "focus_automatic_continuous=0",
    #                 "-c", "focus_absolute=300",
    #             ],
    #         ),
    #     ],
    # )

    return LaunchDescription([
        left_camera,
        right_camera,
        visual_servo_camera,
    ])
