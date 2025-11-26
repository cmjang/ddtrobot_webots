cmake_minimum_required(VERSION 3.8)
project(joy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-function)
  # add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
ament_auto_find_build_dependencies()

include_directories()

generate_parameter_library(joy_controller_parameters
  src/joy_controller_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/joy_controller.cpp
)

target_link_libraries(${PROJECT_NAME}_lib joy_controller_parameters)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
