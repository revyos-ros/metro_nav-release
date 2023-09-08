cmake_minimum_required(VERSION 3.5)
project(base2d_kinematics)

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

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(base2d_kinematics_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(base2d_kinematics src/kinematic_parameters.cpp src/noise_model.cpp)
target_include_directories(base2d_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(base2d_kinematics
  base2d_kinematics_msgs
  rclcpp
  geometry_msgs
  nav_2d_msgs
)

add_executable(kinematics_publisher src/kinematics_publisher.cpp)
target_include_directories(kinematics_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics_publisher base2d_kinematics)
ament_target_dependencies(kinematics_publisher
  base2d_kinematics_msgs
  rclcpp
  geometry_msgs
  nav_2d_msgs
)
install(TARGETS kinematics_publisher
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_base2d_kinematics HAS_LIBRARY_TARGET)
install(TARGETS base2d_kinematics
        EXPORT export_base2d_kinematics
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)
ament_export_dependencies(base2d_kinematics_msgs)

ament_package()
