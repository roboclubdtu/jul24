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
        baxter_tools
)
include_directories(${catkin_INCLUDE_DIRS})
add_service_files(
        FILES
        Capture.srv
        LoadResource.srv
        SaveResource.srv
        StackInfo.srv
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
# Utils macros
# =============================================================================
macro(configure_exec target)
    target_include_directories(${target} PRIVATE "include")
    target_link_libraries(${target} ${catkin_LIBRARIES} jul24_common)
    add_dependencies(${target} ${PROJECT_NAME}_gencpp)
endmacro()

macro(custom_target)
    cmake_parse_arguments(ARGS "" "TARGET" "FILES" ${ARGN})
    message("-- New target: ${ARGS_TARGET}")
    add_executable(${ARGS_TARGET} ${ARGS_FILES} ${ARGS_UNPARSED_ARGUMENTS})
    configure_exec(${ARGS_TARGET})
    install(
            TARGETS ${ARGS_TARGET}
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
    )
endmacro()

macro(install_shell_script file)
    message("-- New script: ${file}")
    install(
            FILES ${file}
            PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
            DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})
endmacro()

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
        include/dtu_jul24/common/BaxterJoint.hpp
        src/common/BaxterJoints.cpp
)
target_include_directories(jul24_common PRIVATE "include")
target_link_libraries(jul24_common ${catkin_LIBRARIES})
add_dependencies(jul24_common ${PROJECT_NAME}_gencpp)

# =============================================================================
# Choreographer: Run everything together and keep the timing right
# =============================================================================
custom_target(
        TARGET choreo
        FILES
        src/choreo/main.cpp
        include/dtu_jul24/choreo/choreograph.hpp
        src/choreo/server_discovery.cpp
        include/dtu_jul24/common/ServerStatus.hpp
        src/choreo/running.cpp
        src/choreo/node_io.cpp
        src/choreo/constructor.cpp

)

# =============================================================================
# Pose Manager: Joint state recorder and player
# =============================================================================
custom_target(TARGET pose_manager
        FILES include/dtu_jul24/pose_manager/pose_manager.hpp
        src/pose_manager/main.cpp
        src/pose_manager/pose_manager.cpp
        include/dtu_jul24/pose_manager/PoseLoader.hpp
        src/pose_manager/pose_loader.cpp
)

# =============================================================================
# Scripts
# =============================================================================
custom_target(TARGET baxter_setup FILES src/scripts/setup_baxter.cpp)
custom_target(TARGET baxter_cmd_duplicate FILES src/scripts/baxter_cmd_dupli.cpp)
install_shell_script(scripts/baxter_init.sh)
install_shell_script(scripts/baxter_stop.sh)

# =============================================================================
# Other files
# =============================================================================
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)