cmake_minimum_required(VERSION 3.5)
project(px4_uwb_locate)

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


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
add_executable(px4_locate_bridge 
  src/px4_locate_bridge.cpp 
  )

target_include_directories(px4_locate_bridge PUBLIC include)

ament_target_dependencies(px4_locate_bridge rclcpp ament_index_cpp geometry_msgs px4_msgs)

install(TARGETS px4_locate_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
