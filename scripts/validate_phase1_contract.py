#!/usr/bin/env python3
"""Fail-fast cross-file contract checks for phase-one mapping/navigation."""

from pathlib import Path
import re
import sys

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
SRC = WORKSPACE / "src"


def load_yaml(path: Path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def close(actual, expected, tolerance=1.0e-9) -> bool:
    return abs(float(actual) - float(expected)) <= tolerance


def main() -> int:
    hardware_path = (
        SRC / "rk3576_footbath_bringup" / "config" / "hardware.yaml"
    )
    hardware = load_yaml(hardware_path)
    geometry = hardware["robot_geometry"]
    stm32 = hardware["stm32"]
    require(
        stm32["transport"] == "RK3576_UART3_M0_to_F407_USART1"
        and stm32["linux_device"] == "/dev/ttyS3"
        and stm32["device"] == "/dev/footbath_stm32",
        "formal STM32 link must use the RK3576 UART3_M0 stable alias",
    )
    require(
        stm32["rk_physical_rx_pin"] == 3
        and stm32["rk_physical_tx_pin"] == 5
        and stm32["rk_ground_pin"] == 6
        and stm32["f407_tx_pin"] == "PA9"
        and stm32["f407_rx_pin"] == "PA10"
        and stm32["fire_dap_role"] == "SWD_only",
        "UART3_M0/F407 wiring metadata changed without review",
    )
    uart_rule = (
        WORKSPACE / "deploy" / "udev" / "99-footbath-uart3-m0.rules"
    ).read_text(encoding="utf-8")
    lidar_rules = (
        WORKSPACE / "deploy" / "udev" / "99-footbath-lidars.rules"
    ).read_text(encoding="utf-8")
    require(
        'KERNEL=="ttyS3"' in uart_rule
        and 'SYMLINK+="footbath_stm32"' in uart_rule,
        "UART3_M0 udev alias rule is missing",
    )
    require(
        'SYMLINK+="footbath_lidar_high"' in lidar_rules
        and 'SYMLINK+="footbath_lidar_low"' in lidar_rules
        and "footbath_lidar_front" not in lidar_rules,
        "dual-lidar udev rules must provide only high/low aliases",
    )
    require(close(geometry["base_radius_m"], 0.22), "body radius must be 0.22 m")
    require(close(geometry["base_height_m"], 0.50), "body height must be 0.50 m")
    require(
        all(close(geometry[key], expected) for key, expected in {
            "laser_high_x_m": 0.10,
            "laser_high_y_m": 0.0,
            "laser_high_z_m": 0.50,
            "laser_low_x_m": 0.10,
            "laser_low_y_m": 0.0,
            "laser_low_z_m": 0.15,
            "laser_high_yaw_rad": 3.1415926536,
            "laser_low_yaw_rad": 3.1415926536,
        }.items()),
        "high/low lidar poses changed without updating the phase-one contract",
    )
    require(
        hardware["rplidar_high"]["scan_topic"] == "/scan_high",
        "high lidar topic must remain /scan_high",
    )
    require(
        hardware["rplidar_low"]["raw_scan_topic"] == "/scan_low_raw"
        and hardware["rplidar_low"]["filtered_scan_topic"] == "/scan_low_front",
        "low lidar raw/filtered topic contract changed",
    )
    require(
        close(hardware["rplidar_low"]["valid_angle_min_rad"], 1.9634954085)
        and close(hardware["rplidar_low"]["valid_angle_max_rad"], -1.9634954085),
        "low lidar raw sector must wrap around +/-pi for the physical front 135 degrees",
    )
    require(
        hardware["distance_sensors"]["front_ultrasonic_hard_stop_enabled"]
        is True,
        "front ultrasonic F407 hard-stop metadata must be enabled",
    )
    require(
        hardware["distance_sensors"]["cliff_hard_stop_enabled"] is False,
        "un-calibrated ToF cliff hard-stop must remain disabled",
    )

    slam = load_yaml(
        SRC / "rk3576_footbath_slam" / "config" / "slam_toolbox_mapping.yaml"
    )
    require(
        slam["slam_toolbox"]["ros__parameters"]["scan_topic"] == "/scan_high",
        "SLAM Toolbox must use only /scan_high",
    )

    nav = load_yaml(
        SRC / "rk3576_footbath_navigation" / "config" / "nav2_params.yaml"
    )
    amcl = nav["amcl"]["ros__parameters"]
    require(amcl["scan_topic"] == "/scan_high", "AMCL must use only /scan_high")
    controller_params = nav["controller_server"]["ros__parameters"]
    controller = controller_params["FollowPath"]
    require(
        float(controller["max_vel_x"]) <= 0.20
        and float(controller["max_speed_xy"]) <= 0.20,
        "DWB automatic linear speed exceeds 0.20 m/s",
    )
    bt = nav["bt_navigator"]["ros__parameters"]
    progress = controller_params["progress_checker"]
    planner = nav["planner_server"]["ros__parameters"]
    require(
        int(bt["bt_loop_duration"]) >= 50
        and int(bt["default_server_timeout"]) >= 500
        and float(controller_params["controller_frequency"]) <= 10.0
        and float(planner["expected_planner_frequency"]) <= 1.0,
        "Nav2 scheduler rates/timeouts regressed to the overload-prone baseline",
    )
    require(
        float(progress["required_movement_radius"]) <= 0.05
        and float(progress["movement_time_allowance"]) >= 30.0,
        "low-speed progress checker became too strict",
    )
    require(
        int(controller["vx_samples"]) <= 12
        and int(controller["vy_samples"]) == 1
        and int(controller["vtheta_samples"]) <= 16,
        "DWB trajectory sampling exceeds the RK3576 load budget",
    )
    smoother = nav["velocity_smoother"]["ros__parameters"]
    require(
        max(abs(float(smoother["max_velocity"][0])),
            abs(float(smoother["min_velocity"][0]))) <= 0.20,
        "velocity smoother linear speed exceeds 0.20 m/s",
    )

    local = nav["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    for name, costmap in (("local", local), ("global", global_map)):
        sources = set(
            costmap["obstacle_layer"]["observation_sources"].split()
        )
        require(
            sources == {"scan_high", "scan_low_front"},
            f"{name} costmap must use high and filtered-low lidars",
        )
        require(
            costmap["obstacle_layer"]["scan_low_front"]["topic"]
            == "/scan_low_front",
            f"{name} costmap bypasses the low-lidar angular filter",
        )
    require(
        "ultrasonic_range_layer" in local["plugins"]
        and local["ultrasonic_range_layer"]["topics"]
        == ["/range/ultrasonic"],
        "front ultrasonic must enter the local costmap",
    )
    require(
        "ultrasonic_range_layer" not in global_map["plugins"],
        "front ultrasonic must not be persisted in the global costmap",
    )

    safety = load_yaml(
        SRC / "rk3576_footbath_safety" / "config" / "safety.yaml"
    )
    limiter = safety["auto_cmd_vel_limiter"]["ros__parameters"]
    mux = safety["footbath_command_mux"]["ros__parameters"]
    require(
        limiter["input_topic"] == "/cmd_vel"
        and limiter["output_topic"] == "/cmd_vel_auto_limited"
        and limiter["lease_topic"] == "/safety/auto_motion_lease"
        and float(limiter["max_linear_mps"]) <= 0.20,
        "automatic limiter topic/lease/speed contract changed",
    )
    require(
        mux["topics"] == {
            "manual": "/cmd_vel_manual",
            "auto": "/cmd_vel_auto_limited",
            "output": "/cmd_vel_selected",
        },
        "command mux topic contract changed",
    )
    require(
        close(mux["manual_max_linear_mps"], 1.00)
        and float(mux["auto_max_linear_mps"]) <= 0.20,
        "manual/automatic mux limits changed",
    )

    bridge = load_yaml(
        SRC / "rk3576_footbath_base" / "config" / "serial_bridge.yaml"
    )["stm32_serial_bridge"]["ros__parameters"]
    require(
        bridge["topics"]["cmd_vel"] == "/cmd_vel_selected",
        "STM32 bridge must subscribe only to /cmd_vel_selected",
    )
    require(
        close(bridge["safety"]["max_linear_mps"], 1.00),
        "STM32 bridge manual software maximum must be 1.00 m/s",
    )

    auto_launch = (
        SRC
        / "rk3576_footbath_bringup"
        / "launch"
        / "auto_mapping.launch.py"
    ).read_text(encoding="utf-8")
    hardware_launch = (
        SRC / "rk3576_footbath_bringup" / "launch" / "hardware.launch.py"
    ).read_text(encoding="utf-8")
    require(
        '"start_auto_limiter": "true"' in auto_launch
        and '"auto_limiter_require_lease": "true"' in auto_launch,
        "auto_mapping must start the leased automatic limiter",
    )
    require(
        '"start_command_mux": "true"' in hardware_launch,
        "hardware launch must start the final command mux",
    )

    foxglove = (
        SRC
        / "rk3576_footbath_remote"
        / "scripts"
        / "footbath_visualization_bridge"
    ).read_text(encoding="utf-8")
    require(
        '"^/cmd_vel_foxglove$"' in foxglove
        and '"clientPublish"' in foxglove,
        "Foxglove manual mapping input is not narrowly enabled",
    )

    monorepo = WORKSPACE.parents[1]
    board_config = monorepo / "Chassis" / "include" / "board_config.h"
    if board_config.is_file():
        text = board_config.read_text(encoding="utf-8")
        for macro, expected in {
            "PS2_MANUAL_MAX_LINEAR_MPS": 0.30,
            "MAX_LINEAR_MPS": 1.00,
            "MAX_WHEEL_MPS": 1.00,
        }.items():
            match = re.search(
                rf"^#define\s+{macro}\s+([0-9.]+)f",
                text,
                flags=re.MULTILINE,
            )
            require(match is not None, f"missing F407 macro {macro}")
            require(
                close(match.group(1), expected),
                f"F407 macro {macro} must be {expected:.2f} m/s",
            )
        require(
            re.search(
                r"^#define\s+FRONT_OBSTACLE_SAFETY_ENABLE\s+1U",
                text,
                flags=re.MULTILINE,
            ) is not None,
            "F407 front ultrasonic hard-stop must remain enabled",
        )

    print("PASS: phase-one geometry, sensor roles and speed chain are consistent")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, KeyError, TypeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
