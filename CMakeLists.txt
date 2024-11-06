cmake_minimum_required(VERSION 3.26)
project(raytracer)

set(CMAKE_CXX_STANDARD 20)

# Collect all header files
file(GLOB_RECURSE HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

# Create executable with main.cpp and specify include directory
add_executable(${PROJECT_NAME} src/main.cpp ${HEADERS})

# Add include directories
target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)