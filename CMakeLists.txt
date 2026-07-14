cmake_minimum_required(VERSION 3.16)
project(droneMission VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVSDK_PKG REQUIRED mavsdk)

find_package(yaml-cpp REQUIRED)

set(SOURCE_FILES
    src/main.cpp
    src/droneLink.cpp
    src/flightOps.cpp
    src/offboardControl.cpp
    src/servoControl.cpp
    src/pidController.cpp
    src/visionInterface.cpp
    src/missionConfig.cpp
    src/missionStateMachine.cpp
)

set(HEADER_FILES
    include/types.h
    include/droneLink.h
    include/flightOps.h
    include/offboardControl.h
    include/servoControl.h
    include/pidController.h
    include/visionInterface.h
    include/missionConfig.h
    include/missionStateMachine.h
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${MAVSDK_PKG_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    yaml-cpp::yaml-cpp
    ${MAVSDK_PKG_LIBRARIES}
    pthread
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MAVSDK_PKG_CFLAGS}
    ${MAVSDK_PKG_CFLAGS_OTHER}
    -Wall -O2
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG)
endif()
