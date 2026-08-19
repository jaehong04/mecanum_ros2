cmake_minimum_required(VERSION 3.8)
project(mecanum_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    urdf
    rviz
    meshes
    maps
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/mecanum_keyboard
  scripts/witmotion_ble_imu
  scripts/cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
