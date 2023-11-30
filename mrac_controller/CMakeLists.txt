cmake_minimum_required(VERSION 3.14)
project(mrac_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(MRAC_CONTROLLER_LIB_SRC
  src/mrac_controller.cpp
)

set(MRAC_CONTROLLER_LIB_HEADERS
  include/mrac_controller/mrac_controller.hpp
  include/mrac_controller/visibility_control.hpp
  ${Eigen3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${MRAC_CONTROLLER_LIB_SRC}
  ${MRAC_CONTROLLER_LIB_HEADERS}
)

set(MRAC_CONTROLLER_NODE_SRC
  src/mrac_controller_node.cpp
)

set(MRAC_CONTROLLER_NODE_HEADERS
  include/mrac_controller/mrac_controller_node.hpp
  ${Eigen3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${MRAC_CONTROLLER_NODE_SRC}
  ${MRAC_CONTROLLER_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "mrac_controller::MracControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_mrac_controller
    test/test_mrac_controller.cpp
  )
  ament_target_dependencies(test_mrac_controller ${TEST_SOURCES})
  target_link_libraries(test_mrac_controller ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
