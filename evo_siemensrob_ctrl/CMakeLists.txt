cmake_minimum_required(VERSION 3.5)
project(evo_siemensrob_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(rib_support
	rib_support/include
)

include_directories(
  include
)

add_subdirectory(rib_support)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/plc_interface_node.cpp
                                    src/SWCPU_data_exchange.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs std_srvs sensor_msgs nav_msgs tf2_geometry_msgs tf2_ros geometry_msgs)

target_link_libraries(${PROJECT_NAME}_node
  #PRIVATE
    rib_support
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
