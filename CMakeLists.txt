cmake_minimum_required(VERSION 3.16)
project(droneMission VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVSDK_PKG REQUIRED mavsdk)

find_package(yaml-cpp REQUIRED)

set(SOURCE_FILES
    src/mission/main.cpp
    src/comm/droneLink.cpp
    src/comm/flightOps.cpp
    src/comm/offboardControl.cpp
    src/comm/servoControl.cpp
    src/control/pidController.cpp
    src/control/dualLoopPid.cpp
    src/vision/visionInterface.cpp
    src/mission/missionConfig.cpp
    src/mission/missionStateMachine.cpp
    src/bomb/coordinateMapper.cpp
    src/bomb/kalmanFilter.cpp
    src/bomb/targetTracker.cpp
    src/bomb/bombDropSystem.cpp
)

set(HEADER_FILES
    include/mission/types.h
    include/comm/droneLink.h
    include/comm/flightOps.h
    include/comm/offboardControl.h
    include/comm/servoControl.h
    include/control/pidController.h
    include/control/dualLoopPid.h
    include/vision/visionInterface.h
    include/mission/missionConfig.h
    include/mission/missionStateMachine.h
    include/bomb/coordinateMapper.h
    include/bomb/kalmanFilter.h
    include/bomb/targetTracker.h
    include/bomb/bombDropSystem.h
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/mission
    ${PROJECT_SOURCE_DIR}/include/comm
    ${PROJECT_SOURCE_DIR}/include/control
    ${PROJECT_SOURCE_DIR}/include/vision
    ${PROJECT_SOURCE_DIR}/include/bomb
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
