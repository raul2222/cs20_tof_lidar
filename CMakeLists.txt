cmake_minimum_required(VERSION 3.5)
project(cs20_tof_lidar)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(TARGET_NAME cs20l)

message("configure ${TARGET_NAME}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fopenmp")

include_directories(include)
link_directories(lib/x64)



file(GLOB SRC src/*.cpp)
message("configure ${SRC}")
add_executable(cs20 ${SRC})


ament_target_dependencies(cs20 PUBLIC rclcpp std_msgs sensor_msgs)

target_link_libraries(cs20 PRIVATE cstreamer)

install( TARGETS 
  cs20
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
