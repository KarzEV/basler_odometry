cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)

project(basler_odometry)

set(BASLER_PUBLISHER "basler_publisher")
set(VISUAL_ODOMETER "visual_odometer")
set(VISUAL_ODOMETER_LIB "visual_odometer_lib")

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
find_package(
        Boost
        COMPONENTS filesystem
)

add_message_files(
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

add_executable(
        ${BASLER_PUBLISHER}
        src/pylon_wrapper.cpp
        src/basler_publisher.cpp
)

target_link_libraries(
        ${BASLER_PUBLISHER}
        ${catkin_LIBRARIES}
        ${Pylon_LIBRARIES})

add_dependencies(${BASLER_PUBLISHER} basler_odometry_generate_messages_cpp)

install(
        TARGETS ${BASLER_PUBLISHER}

        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(${VISUAL_ODOMETER_LIB} STATIC
        src/odometry_lib/bucket.cpp
        src/odometry_lib/feature.cpp
        src/odometry_lib/utils.cpp
        src/odometry_lib/visualOdometry.cpp)

target_include_directories(${VISUAL_ODOMETER_LIB} PUBLIC include/basler_odometry/odometry_lib)

add_executable(
        ${VISUAL_ODOMETER}
        src/odometry_handler.cpp
        src/odometry_node.cpp
        src/visual_odometer.cpp
)

target_link_libraries(
        ${VISUAL_ODOMETER}
        ${catkin_LIBRARIES}
        ${VISUAL_ODOMETER_LIB}
        Boost::filesystem
)

add_dependencies(${VISUAL_ODOMETER} basler_odometry_generate_messages_cpp)

install(
        TARGETS ${VISUAL_ODOMETER}

        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)