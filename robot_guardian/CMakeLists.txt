cmake_minimum_required(VERSION 3.8)
project(robot_guardian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(DIAGNOSTIC_COMPONENTS
  robot_guardian
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED src/robot_guardian/robot_guardian.cpp)
# rclcpp_components_register_nodes(robot_guardian "robot_guardian::RobotGuardian")

ament_auto_add_executable(${PROJECT_NAME}
  src/main.cpp
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_auto_find_test_dependencies()
  # ament_lint_auto_find_test_dependencies()

  # foreach(component ${DIAGNOSTIC_COMPONENTS})
  #   ament_auto_add_gtest(${component}_test
  #     tests/${component}_test.cpp
  #   )
  # endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
