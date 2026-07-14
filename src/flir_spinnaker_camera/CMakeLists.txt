cmake_minimum_required(VERSION 3.8)
project(flir_spinnaker_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlirMetadata.msg"
  DEPENDENCIES std_msgs
)

set(SPINNAKER_ROOT "$ENV{SPINNAKER_ROOT}" CACHE PATH "Spinnaker SDK root directory")

set(_spinnaker_include_hints)
set(_spinnaker_library_hints)
if(SPINNAKER_ROOT)
  list(APPEND _spinnaker_include_hints
    "${SPINNAKER_ROOT}"
    "${SPINNAKER_ROOT}/include"
  )
  list(APPEND _spinnaker_library_hints
    "${SPINNAKER_ROOT}"
    "${SPINNAKER_ROOT}/lib"
    "${SPINNAKER_ROOT}/lib64"
  )
endif()

# SPINNAKER_ROOT must win over CMAKE_PREFIX_PATH. ROS distributions can ship a
# vendored libSpinnaker (ros-humble-spinnaker-camera-driver installs 4.2 into
# /opt/ros/humble/lib), and find_library searches CMAKE_PREFIX_PATH before
# HINTS. Without this first pass the headers and the library come from
# different SDKs, and Spinnaker fails at runtime with
# "System instance has not been initialized [-1002]".
if(SPINNAKER_ROOT)
  find_path(SPINNAKER_INCLUDE_DIR
    NAMES Spinnaker.h
    HINTS ${_spinnaker_include_hints}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )

  find_library(SPINNAKER_LIBRARY
    NAMES Spinnaker
    HINTS ${_spinnaker_library_hints}
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
  )
endif()

find_path(SPINNAKER_INCLUDE_DIR
  NAMES Spinnaker.h
  HINTS ${_spinnaker_include_hints}
  PATH_SUFFIXES include
)

find_library(SPINNAKER_LIBRARY
  NAMES Spinnaker
  HINTS ${_spinnaker_library_hints}
  PATH_SUFFIXES lib lib64
)

if(NOT SPINNAKER_INCLUDE_DIR OR NOT SPINNAKER_LIBRARY)
  message(FATAL_ERROR
    "Spinnaker SDK not found. Set SPINNAKER_ROOT or make the SDK discoverable via CMAKE_PREFIX_PATH."
  )
endif()

get_filename_component(SPINNAKER_LIBRARY_DIR ${SPINNAKER_LIBRARY} DIRECTORY)

add_executable(flir_spinnaker_camera_node
  src/flir_spinnaker_camera_node.cpp
)

add_executable(flir_camera_extrinsics_tf_node
  src/flir_camera_extrinsics_tf_node.cpp
)

add_executable(flir_multicam_inventory_tool
  src/flir_multicam_inventory_tool.cpp
)

target_compile_features(flir_spinnaker_camera_node PUBLIC cxx_std_17)
target_compile_features(flir_camera_extrinsics_tf_node PUBLIC cxx_std_17)
target_compile_features(flir_multicam_inventory_tool PUBLIC cxx_std_17)

target_include_directories(flir_spinnaker_camera_node SYSTEM PRIVATE
  ${SPINNAKER_INCLUDE_DIR}
)

target_include_directories(flir_multicam_inventory_tool SYSTEM PRIVATE
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

target_link_libraries(flir_multicam_inventory_tool
  ${SPINNAKER_LIBRARY}
)

# --disable-new-dtags emits DT_RPATH rather than DT_RUNPATH. Only DT_RPATH is
# searched ahead of LD_LIBRARY_PATH, and ROS's setup.bash puts
# /opt/ros/humble/lib (which may hold a different vendored libSpinnaker) on
# LD_LIBRARY_PATH. Without this, SPINNAKER_LIBRARY_DIR loses at runtime even
# when it wins at link time.
set_target_properties(flir_spinnaker_camera_node PROPERTIES
  BUILD_RPATH ${SPINNAKER_LIBRARY_DIR}
  INSTALL_RPATH ${SPINNAKER_LIBRARY_DIR}
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

set_target_properties(flir_multicam_inventory_tool PROPERTIES
  BUILD_RPATH ${SPINNAKER_LIBRARY_DIR}
  INSTALL_RPATH ${SPINNAKER_LIBRARY_DIR}
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

ament_target_dependencies(flir_spinnaker_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(flir_camera_extrinsics_tf_node
  geometry_msgs
  rclcpp
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(flir_spinnaker_camera_node ${cpp_typesupport_target})

install(TARGETS
  flir_spinnaker_camera_node
  flir_camera_extrinsics_tf_node
  flir_multicam_inventory_tool
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
