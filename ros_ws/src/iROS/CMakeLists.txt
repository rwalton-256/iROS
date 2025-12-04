cmake_minimum_required(VERSION 3.8)
project(iROS)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Resolve symlink and copy the actual file content
get_filename_component(IPHONE_PY_REAL python/iphone.py REALPATH)
configure_file(${IPHONE_PY_REAL} ${CMAKE_CURRENT_BINARY_DIR}/iphone.py COPYONLY)

install(
  PROGRAMS
    python/iphone_interface.py
    python/yolo_node.py
    ${CMAKE_CURRENT_BINARY_DIR}/iphone.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
