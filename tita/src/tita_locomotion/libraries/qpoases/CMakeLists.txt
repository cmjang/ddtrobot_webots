cmake_minimum_required(VERSION 3.5)
project(qpoases)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 添加编译选项以禁用 -Wformat-truncation 警告
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-format-truncation)
    add_compile_options(-Wno-misleading-indentation)
    add_compile_options(-Wno-unused-parameter)
endif()
add_definitions(-D__NO_COPYRIGHT__) 
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# define library
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/BLASReplacement.cpp
    src/Bounds.cpp
    src/Constraints.cpp
    src/Flipper.cpp
    src/Indexlist.cpp
    src/LAPACKReplacement.cpp
    src/Matrices.cpp
    src/MessageHandling.cpp
    src/Options.cpp
    src/OQPinterface.cpp
    src/QProblem.cpp
    src/QProblemB.cpp
    src/SolutionAnalysis.cpp
    src/SparseSolver.cpp
    src/SQProblem.cpp
    src/SQProblemSchur.cpp
    src/SubjectTo.cpp
    src/Utils.cpp
)

# define examples
option(QPOASES_BUILD_EXAMPLES "Build examples." OFF)
if(QPOASES_BUILD_EXAMPLES)
  set(EXAMPLE_NAMES
      example1
      example1a
      example1b
      example2
      example3
      example3b
      example4
      example5
      exampleLP
      qrecipe
      qrecipeSchur
  )

  foreach(ELEMENT ${EXAMPLE_NAMES})
    add_executable(${ELEMENT} examples/${ELEMENT}.cpp)
    target_link_libraries(${ELEMENT} ${PROJECT_NAME})
  endforeach(ELEMENT ${EXAMPLE_NAMES})
endif()

# testing
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_auto_package()
