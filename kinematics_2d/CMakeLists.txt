cmake_minimum_required(VERSION 3.5)
project(kinematics_2d)

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
find_package(kinematics_2d_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(kinematics_2d src/kinematic_parameters.cpp src/noise_model.cpp)
target_include_directories(kinematics_2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kinematics_2d
  kinematics_2d_msgs
  rclcpp
  geometry_msgs
  nav_2d_msgs
)

add_executable(kinematics_publisher src/kinematics_publisher.cpp)
target_include_directories(kinematics_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics_publisher kinematics_2d)
ament_target_dependencies(kinematics_publisher
  kinematics_2d_msgs
  rclcpp
  geometry_msgs
  nav_2d_msgs
)
install(TARGETS kinematics_publisher
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_kinematics_2d HAS_LIBRARY_TARGET)
install(TARGETS kinematics_2d
        EXPORT export_kinematics_2d
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)
ament_export_dependencies(kinematics_2d_msgs)

ament_package()
