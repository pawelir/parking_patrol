cmake_minimum_required(VERSION 3.8)
project(parking_patrol_simulation)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(GCC_COVERAGE_COMPILE_FLAGS "-pthread -DBOOST_ERROR_CODE_HEADER_ONLY -BOOST_SYSTEM_NO_DEPRECATED")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)

include_directories(include)

install(DIRECTORY
  launch
  world
  DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
