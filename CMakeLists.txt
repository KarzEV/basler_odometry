cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

project(basler_odometry)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif ()

if (NOT Pylon_FOUND)
    message(FATAL_ERROR "Don't find Pylon")
endif ()

find_package(
        catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        sensor_msgs
        std_msgs
        message_generation
)

add_message_files(
        DIRECTORY msg
        FILES
        Basler.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

catkin_package(
        DEPENDS message_runtime
)

include_directories(
        include/basler_odometry
        ${catkin_INCLUDE_DIRS}
        ${Pylon_INCLUDE_DIRS}
)

add_executable(basler_odometry
        src/pylon_wrapper.cpp
        src/basler_publisher.cpp
        )

target_link_libraries(
        basler_odometry
        ${catkin_LIBRARIES}
        ${Pylon_LIBRARIES})

#add_dependencies(basler_odometry basler_publisher_generate_messages_cpp)