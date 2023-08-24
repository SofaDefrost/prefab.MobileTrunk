cmake_minimum_required(VERSION 3.5)
project(somorob)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(summit_xl src/summit_xl.cpp)

target_include_directories(summit_xl PRIVATE ${geometry_msgs_INCLUDE_DIRS})

ament_target_dependencies(summit_xl rclcpp geometry_msgs)


install(TARGETS
  summit_xl
  DESTINATION lib/${PROJECT_NAME})


ament_package()
