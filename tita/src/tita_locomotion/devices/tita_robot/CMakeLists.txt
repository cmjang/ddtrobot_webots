cmake_minimum_required(VERSION 3.8)
project(tita_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)
set(SRC_FILES
    src/tita_robot.cpp
    src/can_receiver.cpp
    src/can_sender.cpp
)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SRC_FILES} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()



# cmake_minimum_required(VERSION 3.10)
# project(tita_robot VERSION 1.0)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# find_package(ament_cmake REQUIRED) 
# # Include directories
# include_directories(include)

# # Source files
# set(SRC_FILES
#     src/tita_robot.cpp
#     src/can_receiver.cpp
#     src/can_sender.cpp
# )

# # Library target
# add_library(tita_robot SHARED ${SRC_FILES})

# install(
#     TARGETS tita_robot
#     LIBRARY DESTINATION lib
# )

# install(
#     DIRECTORY include
#     DESTINATION include/
# )

# # Test executable
# add_executable(test_executable test/test.cpp)
# target_link_libraries(test_executable tita_robot)


# ament_export_include_directories(include)
# ament_export_libraries(tita_robot)
# ament_package()


# Specify installation paths for headers and libraries
# 安装目标库
# install(TARGETS tita_robot
#         EXPORT tita_robotTargets
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin
#         INCLUDES DESTINATION include)

# 安装头文件
# install(DIRECTORY include/ DESTINATION include)

# 生成并安装 Config 文件
# install(EXPORT tita_robotTargets
#         FILE tita_robotConfig.cmake
#         DESTINATION lib/cmake/tita_robot)

# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#     "${CMAKE_CURRENT_BINARY_DIR}/tita_robotConfigVersion.cmake"
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
# )

# configure_file(tita_robotConfig.cmake.in
#     "${CMAKE_CURRENT_BINARY_DIR}/tita_robotConfig.cmake" @ONLY)

# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/tita_robotConfig.cmake"
#     "${CMAKE_CURRENT_BINARY_DIR}/tita_robotConfigVersion.cmake"
#     DESTINATION lib/cmake/tita_robot
# )

# # 创建 uninstall 目标
# add_custom_target(uninstall
#     COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)
    
# configure_file(
#     "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
#     "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
#     IMMEDIATE @ONLY)

