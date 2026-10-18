cmake_minimum_required(VERSION 3.5)
project(tetra_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
# 웹 관제가 로봇 위치로 쓰는 /footprint 를 TF 에서 만들어 발행한다
install(PROGRAMS scripts/robot_pose_publisher.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
