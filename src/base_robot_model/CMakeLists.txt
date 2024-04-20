cmake_minimum_required(VERSION 3.8)
project(base_robot_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(library_name base_robot_model_package)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(dependencies
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  nav2_util
  glog
  visualization_msgs
  search_example
)

foreach(name IN LISTS dependencies)
  find_package(${name} REQUIRED)
endforeach()

include_directories(include)

add_library(${library_name} SHARED
  src/robot_model.cpp
)

target_link_libraries(${library_name} 
  ${GLOG_LIBRARIES}
  glog::glog
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC 
  ${GLOG_INCLUDE_DIRS}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
