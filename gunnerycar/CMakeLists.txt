cmake_minimum_required(VERSION 3.5)
project(gunnerycar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

add_library(gunnerycar_follow_path include/gunnerycar_follow_path.h src/gunnerycar_follow_path.cpp)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(twist_transform src/cmd_vel_transfrom.cpp)
add_executable(init_pose_node src/init_pose_node.cpp)
add_executable(init_target_node src/nav_to_pose.cpp)
add_executable(follow_path_node src/follow_path_node.cpp)

ament_target_dependencies(twist_transform
  rclcpp
  geometry_msgs
)

ament_target_dependencies(init_pose_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(init_target_node
  rclcpp
  rclcpp_action    
  nav2_msgs        
  geometry_msgs    
  tf2             
  tf2_geometry_msgs 
  tf2_ros
)

ament_target_dependencies(gunnerycar_follow_path
  rclcpp
  rclcpp_action    
  nav2_msgs        
  geometry_msgs    
  tf2             
  tf2_geometry_msgs 
  tf2_ros
)

target_link_libraries(follow_path_node gunnerycar_follow_path)

install(DIRECTORY launch urdf config world map
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include
 DESTINATION include
)

install(TARGETS twist_transform
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS init_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS init_target_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS follow_path_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
