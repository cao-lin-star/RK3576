cmake_minimum_required(VERSION 3.8)
project(rk3576_footbath_remote)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/footbath_remote_status
  scripts/footbath_collect_debug
  scripts/footbath_motion_arm
  scripts/footbath_safe_drive
  scripts/footbath_safe_drive.py
  scripts/footbath_uart4_console
  scripts/footbath_uart4_console.py
  scripts/footbath_loaded_deadband
  scripts/footbath_loaded_deadband.py
  scripts/footbath_loaded_pid_test
  scripts/footbath_flash_f407
  scripts/footbath_visualization_bridge
  scripts/footbath_foxglove_drive_gate.py
  scripts/install_remote_debug.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  deploy
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
