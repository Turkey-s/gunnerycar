cmake_minimum_required(VERSION 3.5)
project(autofleet)

SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

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

add_definitions(-DTESTING)

# 所有依赖项
set(dependencies
  ament_cmake
  rclcpp
  geometry_msgs
  behaviortree_cpp_v3
  rclcpp_action
  nav2_msgs        
  tf2             
  tf2_geometry_msgs 
  tf2_ros
  nav_msgs
  std_msgs
  std_srvs
  sensor_msgs
)

foreach(package IN LISTS dependencies)
    find_package(${package} REQUIRED)
endforeach()

#头文件目录(如果有子目录，需要注意src引用时需要从include开始)
include_directories(
  include
)

# 添加共享库文件
add_library(autofleet_node SHARED include/autofleet/autofleet_node.hpp src/autofleet_node.cpp)
ament_target_dependencies(autofleet_node
  ${dependencies}
)

add_library(MoveState SHARED include/autofleet/moveState.hpp src/moveState.cpp)
list(APPEND plugin_libs MoveState)
add_library(FormTeamState SHARED include/autofleet/formTeamState.hpp src/formTeamState.cpp)
list(APPEND plugin_libs FormTeamState)
add_library(BreakTeamState SHARED include/autofleet/breakTeamState.hpp src/breakTeamState.cpp)
list(APPEND plugin_libs BreakTeamState)



foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${bt_plugin}
    autofleet_node
  )
endforeach()

#添加执行库文件
add_executable(autofleet_node_exec src/main.cpp)

add_executable(mainTest test/mainTest.cpp)

add_executable(laser_filter_node src/laserScan_filter.cpp)

target_link_libraries(autofleet_node_exec
  autofleet_node
)

ament_target_dependencies(autofleet_node_exec
  ${dependencies}
)

ament_target_dependencies(mainTest
  ${dependencies}
)

ament_target_dependencies(laser_filter_node
  ${dependencies}
)

#添加install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include
 DESTINATION include
)

#ROS2 run 是在 lib/autofleet 下找可执行文件
install(TARGETS autofleet_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mainTest
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS laser_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

#行为树在loading 插件时是在lib下找
install(TARGETS ${plugin_libs} autofleet_node
  DESTINATION lib
)

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

ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
