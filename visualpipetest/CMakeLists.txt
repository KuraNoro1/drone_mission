cmake_minimum_required(VERSION 3.16)
project(visualpipetest VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE pthread)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -O2)
