cmake_minimum_required(VERSION 3.8)
project(gpd_ros2)

cmake_policy(SET CMP0074 NEW)

# -------- Options / defaults (tweak if your GPD install differs) ----------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Match ROS1 flags: -O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes
add_compile_options(-O3 -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes)

# Let the user pass include dir if GPD doesn't install a CMake config
set(GPD_LIB_INCLUDE_DIR "" CACHE PATH "Path to GPD include directory (e.g., /usr/local/include)")

# ------------------------ Find dependencies --------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Our interface package
find_package(gpd_ros2_interfaces REQUIRED)

# Eigen 3
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL 1.9 REQUIRED COMPONENTS common io filters kdtree)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV (used in ROS1 CMake)
find_package(OpenCV REQUIRED)

# OpenMP like ROS1 CMake flags
find_package(OpenMP)
if (OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found: enabling")
endif()

# GPD library (mirror ROS1 logic)
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
  message(STATUS "Library GPD found at: ${GPD_LIB}")
else()
  message(FATAL_ERROR "Library GPD not found. Set LD_LIBRARY_PATH or install GPD to /usr/local/lib.")
endif()

# Try to discover/include GPD headers
if (GPD_LIB_INCLUDE_DIR)
  include_directories(${GPD_LIB_INCLUDE_DIR})
  message(STATUS "Using GPD include dir: ${GPD_LIB_INCLUDE_DIR}")
else()
  # Best-effort guesses
  foreach(HINT IN ITEMS /usr/local/include /usr/include)
    if (EXISTS "${HINT}/gpd")
      include_directories(${HINT})
      message(STATUS "Guessed GPD include dir: ${HINT}")
      break()
    endif()
  endforeach()
endif()

# Our include
include_directories(include)

# ----------------------------- Targets -------------------------------------
# Libraries analogous to ROS1
add_library(${PROJECT_NAME}_grasp_messages
  src/gpd_ros2/grasp_messages.cpp
)
add_library(${PROJECT_NAME}_grasp_plotter
  src/gpd_ros2/grasp_plotter.cpp
)

# Executables analogous to ROS1 names
add_executable(detect_grasps
  src/gpd_ros2/grasp_detection_node.cpp
)
add_executable(detect_grasps_server
  src/gpd_ros2/grasp_detection_server.cpp
)

add_executable(pcd_publisher src/tools/pcd_publisher.cpp)

add_executable(pcd_service_client src/tools/pcd_service_client.cpp)
ament_target_dependencies(pcd_service_client
  rclcpp
  sensor_msgs
  geometry_msgs
  gpd_ros2_interfaces
  pcl_conversions
  PCL
)
install(TARGETS pcd_service_client
  DESTINATION lib/${PROJECT_NAME})

add_executable(mask2gpd src/tools/mask2gpd.cpp)
ament_target_dependencies(mask2gpd
  rclcpp
  sensor_msgs
)
install(TARGETS mask2gpd
  DESTINATION lib/${PROJECT_NAME})


# Include dirs for targets
target_include_directories(${PROJECT_NAME}_grasp_messages PUBLIC include)
target_include_directories(${PROJECT_NAME}_grasp_plotter PUBLIC include)
target_include_directories(detect_grasps PUBLIC include)
target_include_directories(detect_grasps_server PUBLIC include)

# Common linkage
set(COMMON_LINK_LIBS
  ${GPD_LIB}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
if (OpenMP_CXX_FOUND)
  list(APPEND COMMON_LINK_LIBS OpenMP::OpenMP_CXX)
endif()

target_link_libraries(${PROJECT_NAME}_grasp_messages ${COMMON_LINK_LIBS})
target_link_libraries(${PROJECT_NAME}_grasp_plotter  ${COMMON_LINK_LIBS})

ament_target_dependencies(${PROJECT_NAME}_grasp_messages
  rclcpp std_msgs sensor_msgs geometry_msgs pcl_conversions gpd_ros2_interfaces
)
ament_target_dependencies(${PROJECT_NAME}_grasp_plotter
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_conversions gpd_ros2_interfaces
)

ament_target_dependencies(pcd_publisher rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})
install(TARGETS pcd_publisher RUNTIME DESTINATION lib/${PROJECT_NAME})

target_link_libraries(detect_grasps
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  ${COMMON_LINK_LIBS}
)
ament_target_dependencies(detect_grasps
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_conversions gpd_ros2_interfaces
)

target_link_libraries(detect_grasps_server
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  ${COMMON_LINK_LIBS}
)
ament_target_dependencies(detect_grasps_server
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_conversions gpd_ros2_interfaces
)

# ----------------------------- Install -------------------------------------
install(TARGETS
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  detect_grasps
  detect_grasps_server
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_grasp_messages ${PROJECT_NAME}_grasp_plotter)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs
  pcl_conversions gpd_ros2_interfaces
)

ament_package()