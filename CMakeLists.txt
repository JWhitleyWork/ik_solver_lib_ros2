cmake_minimum_required(VERSION 3.5)
project(ik_solver_lib)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(orocos_kdl REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trac_ik_solver.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)