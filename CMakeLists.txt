# reference:
#  - https://github.com/ros2/demos/blob/master/demo_nodes_cpp/CMakeLists.txt
#  - https://github.com/ros2/demos/blob/foxy/dummy_robot/dummy_sensors/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(energy_sensors)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(example_interfaces REQUIRED)
# not used 
#find_package(sensor_msgs REQUIRED)
#https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html

add_executable(energy_sensors 
${CMAKE_CURRENT_SOURCE_DIR}/src/periodic.c
${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_file.c
${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_hwmon.c
${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_iio.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_ina226.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_ina231.c
${CMAKE_CURRENT_SOURCE_DIR}/src/time_utils.c    
${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

add_compile_definitions(UDEV_NOTFOUND)
# will only publish data into the topics. no printing into stout
add_compile_definitions(DONT_PRINT)

ament_target_dependencies(energy_sensors
  "rclcpp"
  "std_msgs"
)

target_include_directories(energy_sensors PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/include
${CMAKE_CURRENT_SOURCE_DIR}/include/private
)

install(TARGETS
  energy_sensors
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif() 


# Flags
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_FLAGS "-Wall -Wextra -Werror")
# set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")
# set(CMAKE_CPP_FLAGS_RELEASE "-DNDEBUG")

# # Targets
# add_executable(sampler)

# # Headers directory
# target_include_directories(sampler PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# # Source files
# target_sources(sampler PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/periodic.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/sample_sensors.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_file.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_hwmon.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_iio.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_ina226.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_ina231.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/time_utils.c
# )

# The udev library is used by the SmartPower sensor.
# Since most boards may not need it and having it installed
# should not be mandatory, it is optional.
# find_library(LIB_UDEV udev)
# if(NOT LIB_UDEV)
#     add_compile_definitions(UDEV_NOTFOUND)
# else()
#     target_sources(energy_sensors PRIVATE
#         ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_smartpower.c
#         ${CMAKE_CURRENT_SOURCE_DIR}/src/hid.c
#     )

#     target_link_libraries(energy_sensors PRIVATE
#         udev
#     )
# endif()


ament_package()
