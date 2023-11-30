cmake_minimum_required(VERSION 3.14)
project(mrac)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(MRAC_LIB_SRC
  src/mrac.cpp
)

set(MRAC_LIB_HEADERS
  include/mrac/mrac.hpp
  include/mrac/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${MRAC_LIB_SRC}
  ${MRAC_LIB_HEADERS}
)

set(MRAC_NODE_SRC
  src/mrac_node.cpp
)

set(MRAC_NODE_HEADERS
  include/mrac/mrac_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${MRAC_NODE_SRC}
  ${MRAC_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "mrac::MracNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_mrac
    test/test_mrac.cpp
  )
  ament_target_dependencies(test_mrac ${TEST_SOURCES})
  target_link_libraries(test_mrac ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
