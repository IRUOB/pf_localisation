cmake_minimum_required(VERSION 3.8)
project(pf_localisation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-fPIC -O3  -msse4 -shared -I/usr/include/python3.10)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(CMAKE_SHARED_LIBRARY_PREFIX "")
add_library(laser_trace SHARED src/laser_trace/laser_trace.cpp)

install(TARGETS laser_trace
    LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/pf_localisation)

install(PROGRAMS
        pf_localisation/node.py
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

# Set up the libraries and header search paths for this target
target_link_libraries(laser_trace ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(laser_trace PRIVATE ${PYTHON_INCLUDE_DIRS})

ament_package()
