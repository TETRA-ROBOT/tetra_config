################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(tetra_navigation2)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(spatio_temporal_voxel_layer REQUIRED)

################################################################################
# Install
################################################################################
# behavior_trees: EDU 전용 BT(후진 복구 제거본)를 $(find-pkg-share) 경로로 찾도록 설치한다
install(
  DIRECTORY behavior_trees launch maps params rviz
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
