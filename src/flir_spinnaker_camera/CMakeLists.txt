cmake_minimum_required(VERSION 3.8)
project(flir_spinnaker_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlirMetadata.msg"
  DEPENDENCIES std_msgs
)

find_path(SPINNAKER_INCLUDE_DIR
  NAMES Spinnaker.h
  PATHS /opt/spinnaker/include
  NO_DEFAULT_PATH
)

find_library(SPINNAKER_LIBRARY
  NAMES Spinnaker
  PATHS /opt/spinnaker/lib
  NO_DEFAULT_PATH
)

if(NOT SPINNAKER_INCLUDE_DIR OR NOT SPINNAKER_LIBRARY)
  message(FATAL_ERROR "Spinnaker SDK not found under /opt/spinnaker")
endif()

add_executable(flir_spinnaker_camera_node
  src/flir_spinnaker_camera_node.cpp
)

target_compile_features(flir_spinnaker_camera_node PUBLIC cxx_std_17)

target_include_directories(flir_spinnaker_camera_node SYSTEM PRIVATE
  ${SPINNAKER_INCLUDE_DIR}
)

target_include_directories(flir_spinnaker_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(flir_spinnaker_camera_node
  ${OpenCV_LIBS}
  ${SPINNAKER_LIBRARY}
  Threads::Threads
)

set_target_properties(flir_spinnaker_camera_node PROPERTIES
  BUILD_RPATH /opt/spinnaker/lib
  INSTALL_RPATH /opt/spinnaker/lib
)

ament_target_dependencies(flir_spinnaker_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(flir_spinnaker_camera_node ${cpp_typesupport_target})

install(TARGETS flir_spinnaker_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
