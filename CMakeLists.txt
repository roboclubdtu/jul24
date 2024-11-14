cmake_minimum_required(VERSION 3.8)
project(dtu_jul24 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif ()

# =============================================================================
# Catkin configuration
# =============================================================================

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        std_msgs
        sensor_msgs
        geometry_msgs
        message_generation
        actionlib
)

include_directories(${catkin_INCLUDE_DIRS})
add_service_files(
        FILES
        Capture.srv
        LoadResource.srv
        SaveResource.srv
)
add_action_files(DIRECTORY action)
generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
)
catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        message_runtime
)

# =============================================================================
# Common library
# =============================================================================
add_library(jul24_common STATIC
        include/dtu_jul24/common/ResourceServer.hpp
        include/dtu_jul24/common/ResourceContainer.hpp
        include/dtu_jul24/common/ResourceContainer.hpp
        include/dtu_jul24/common/CustomJoints.hpp
        src/common/CustomJoints.cpp
        include/dtu_jul24/common/constants.hpp
        include/dtu_jul24/common/ResourceServer.hpp
        src/common/ResourceServer.cpp
        include/dtu_jul24/common/utils.hpp
        src/common/utils.cpp
        include/dtu_jul24/common/ActionType.hpp
)
target_include_directories(jul24_common PRIVATE "include")
target_link_libraries(jul24_common ${catkin_LIBRARIES})
add_dependencies(jul24_common ${PROJECT_NAME}_gencpp)

# =============================================================================
# Choreographer: Run everything together and keep the timing right
# =============================================================================
add_executable(choreo
        src/choreo/main.cpp
        include/dtu_jul24/choreo/choreograph.hpp
        src/choreo/server_discovery.cpp
        include/dtu_jul24/common/ServerStatus.hpp
        src/choreo/running.cpp
        src/choreo/node_io.cpp
        src/choreo/constructor.cpp
)
target_include_directories(choreo PRIVATE "include")
target_link_libraries(choreo ${catkin_LIBRARIES} jul24_common)
add_dependencies(jul24_common ${PROJECT_NAME}_gencpp)

# =============================================================================
# Pose Manager: Joint state recorder and player
# =============================================================================
add_executable(pose_manager
        include/dtu_jul24/pose_manager/pose_manager.hpp
        src/pose_manager/main.cpp
        src/pose_manager/pose_manager.cpp
)
target_include_directories(pose_manager PRIVATE "include")
target_link_libraries(pose_manager ${catkin_LIBRARIES} jul24_common)
add_dependencies(jul24_common ${PROJECT_NAME}_gencpp)