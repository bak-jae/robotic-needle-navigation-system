cmake_minimum_required(VERSION 3.8)
project(ros2_epos_cmd)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# The proprietary Maxon SDK is intentionally not included in this repository.
# EPOS_CMD_ROOT may point to an extracted/installed EposCmdLib directory.
find_path(EPOS_CMD_INCLUDE_DIR
  NAMES Definitions.h
  HINTS "$ENV{EPOS_CMD_ROOT}/include"
  PATHS "/opt/EposCmdLib_6.8.1.0/include"
  DOC "Directory containing the Maxon EPOS Command Library Definitions.h"
)
find_library(EPOS_CMD_LIBRARY
  NAMES EposCmd
  HINTS
    "$ENV{EPOS_CMD_ROOT}/lib/x86_64"
    "$ENV{EPOS_CMD_ROOT}/lib"
  PATHS
    "/opt/EposCmdLib_6.8.1.0/lib/x86_64"
    "/usr/local/lib"
    "/usr/lib"
  DOC "Maxon EPOS Command Library"
)

if(NOT EPOS_CMD_INCLUDE_DIR OR NOT EPOS_CMD_LIBRARY)
  message(FATAL_ERROR
    "Maxon EPOS Command Library not found. Install version 6.8.1.0 or set "
    "EPOS_CMD_ROOT to its installation root.")
endif()

add_executable(epos_motion_bridge_node src/epos_motion_bridge_node.cpp)
target_include_directories(epos_motion_bridge_node PRIVATE ${EPOS_CMD_INCLUDE_DIR})
ament_target_dependencies(epos_motion_bridge_node rclcpp std_msgs)
target_link_libraries(epos_motion_bridge_node ${EPOS_CMD_LIBRARY})

install(TARGETS
  epos_motion_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES
  scripts/preview_move_capture.py
  scripts/needle_slicer_closed_loop_no_offset.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
