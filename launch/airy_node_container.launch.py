# Based on common_sensor_launch/launch/nebula_node_container.launch.py (Autoware 1.9.0).
# The nebula driver node is replaced by rslidar_sdk_node + AiryConverter;
# the preprocessing chain (crop_box self/mirror -> distortion_corrector -> ring_outlier_filter) is unchanged.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import yaml


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    # Pointcloud preprocessor parameters
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_correction_node_param_path").perform(context),
        allow_substs=True,
    )
    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_path").perform(context),
        allow_substs=True,
    )

    # rslidar_sdk is a standalone executable (not a component)
    driver = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("config_file")}],
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    nodes = []

    nodes.append(
        ComposableNode(
            package="ugv_airy_driver",
            plugin="ugv_airy_driver::AiryConverter",
            name="airy_converter",
            parameters=[{"imu_frame_id": LaunchConfiguration("imu_frame_id")}],
            remappings=[
                ("input/pointcloud", "rslidar_points"),
                ("output/pointcloud", "pointcloud_raw_ex"),
                ("input/imu", "rslidar_imu_data"),
                ("output/imu", LaunchConfiguration("imu_output_topic")),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True
    cropbox_parameters["processing_time_threshold_sec"] = 0.01

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_corrector_node_param],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Ring Outlier Filter is the last component in the pipeline, so control the output frame here
    if LaunchConfiguration("output_as_sensor_frame").perform(context).lower() == "true":
        ring_outlier_output_frame = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        ring_outlier_output_frame = {"output_frame": ""}  # keep the output frame as the input frame
    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", LaunchConfiguration("output_topic")),
            ],
            parameters=[ring_outlier_filter_node_param, ring_outlier_output_frame],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    return [driver, container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg(
        "config_file",
        PathJoinSubstitution([FindPackageShare("ugv_airy_driver"), "config", "airy_rslidar.yaml"]),
        "rslidar_sdk config file (its ros_frame_id must equal frame_id)",
    )
    add_launch_arg("imu_frame_id", "rs_airy/imu_link", "frame id of the Airy built-in IMU")
    add_launch_arg("imu_output_topic", "/sensing/imu/rs_airy/imu_raw", "converted IMU topic")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("frame_id", "rs_airy_top", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("container_name", "airy_node_container")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg("output_topic", "pointcloud_before_sync", "final output topic of the pipeline")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg(
        "distortion_correction_node_param_path",
        PathJoinSubstitution(
            [
                FindPackageShare("common_sensor_launch"),
                "config",
                "distortion_corrector_node.param.yaml",
            ]
        ),
        description="path to parameter file of distortion correction node",
    )
    add_launch_arg(
        "ring_outlier_filter_node_param_path",
        PathJoinSubstitution(
            [
                FindPackageShare("common_sensor_launch"),
                "config",
                "ring_outlier_filter_node.param.yaml",
            ]
        ),
        description="path to parameter file of ring outlier filter node",
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
