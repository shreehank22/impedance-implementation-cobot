find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

if($ENV{USE_COMM_TYPE} STREQUAL "ros")
    find_package(roscpp REQUIRED)
    include_directories(
        ${roscpp_INCLUDE_DIRS}
    )
endif()

include_directories(
    include
    ${COMMON_INSTALL_DIR}/include
)

if(INSTALL_HEADERS_ONLY)
if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    add_definitions(-DUSE_ROS2_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros")
    add_definitions(-DUSE_ROS_COMM)
endif()
    install(DIRECTORY include/
        DESTINATION ${COMMON_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h*"
    )

else()

add_executable(low_bridge src/low_bridge.cpp)

if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    add_definitions(-DUSE_ROS2_COMM)
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros")
    add_definitions(-DUSE_ROS_COMM)
endif()

target_link_libraries(low_bridge
	quad_communication
	Threads::Threads
	yaml-cpp::yaml-cpp
)

if($ENV{USE_COMM_TYPE} STREQUAL "dds")
    target_link_libraries(low_bridge
        dds_comm
    )
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
    target_link_libraries(low_bridge
        ros_comm
    )
elseif($ENV{USE_COMM_TYPE} STREQUAL "ros")
    target_link_libraries(low_bridge
        ros1_comm
    )
endif()

find_package(CycloneDDS-CXX REQUIRED)
# add_executable(indicator_interface
#                 src/indicator_interface.cpp)
# target_link_libraries(indicator_interface
#     quad_comm_msgs
#     CycloneDDS-CXX::ddscxx
#     pigpio
# )

install(TARGETS 
    low_bridge
    # indicator_interface
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)

    # Add new executable for moteus_interface_shm
    add_executable(low_interface
        src/run_low_interface.cpp
        src/low_interface_shm.cpp
        src/pi3hat.cc
    )

    # Set compiler flags
    set_target_properties(low_interface PROPERTIES
        # COMPILE_FLAGS "-O2 -g -Wall -Wno-psabi --std=c++11 -march=armv8-a"
        # More flexible:
        COMPILE_FLAGS "-O0 -g -Wall -Wno-psabi --std=c++11 -march=armv8-a"

    )

    # Set include directories
    target_include_directories(low_interface PRIVATE
        include
        /opt/vc/include
    )

    # Set link directories and libraries
    target_link_directories(low_interface PRIVATE /opt/vc/lib)
    target_link_libraries(low_interface
        bcm_host
        pthread
        yaml-cpp::yaml-cpp
    )

    # Install the new executable
    install(TARGETS low_interface
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
    )

endif()
