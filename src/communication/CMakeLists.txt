# Communication module - supports multiple backends

# Determine communication type
set(COMM_TYPE "dds")  # Default to DDS
if(DEFINED ENV{USE_COMM_TYPE})
    set(COMM_TYPE "$ENV{USE_COMM_TYPE}")
endif()

# Find packages based on communication type
if(COMM_TYPE STREQUAL "ros")
    find_package(roscpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
elseif(COMM_TYPE STREQUAL "ros2")
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
elseif(COMM_TYPE STREQUAL "dds")
    find_package(CycloneDDS-CXX REQUIRED)
endif()

find_package(Threads REQUIRED)

include_directories(
    msgs
    include
    ${COMMON_INSTALL_DIR}/include
)

if(COMM_TYPE STREQUAL "dds")
    include_directories(${COMMON_INSTALL_DIR}/include/ddscxx)
    include_directories(${CycloneDDS-CXX_INCLUDE_DIR})
elseif(COMM_TYPE STREQUAL "ros2")
    set(ROS_INCLUDE /opt/ros/$ENV{ROS_DISTRO}/include)
    include_directories(${ROS_INCLUDE})
elseif(COMM_TYPE STREQUAL "ros")
    set(ROS_INCLUDE
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
    )
    include_directories(${ROS_INCLUDE})
endif()

if(INSTALL_HEADERS_ONLY)
    install(DIRECTORY include/
        DESTINATION ${COMMON_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h*"
    )

    if(COMM_TYPE STREQUAL "dds")
        file(GLOB ROS2_MSG_FILES msgs/idl/ros2/*.idl)
        idlcxx_generate(TARGET quad_comm_msgs
        FILES
        msgs/idl/FloatScalar.idl
        msgs/idl/Point.idl
        msgs/idl/JointData.idl
        msgs/idl/SensorData.idl
        msgs/idl/JoyData.idl
        msgs/idl/QuadLog.idl
        msgs/idl/SolverStats.idl
        msgs/idl/CliData.idl
        msgs/idl/IndicatorData.idl
        ${ROS2_MSG_FILES}
        WARNINGS no-implicit-extensibility)

        set(GENERATED_HEADERS_DIR "${CMAKE_CURRENT_BINARY_DIR}")

        # Install headers
        install(DIRECTORY "${GENERATED_HEADERS_DIR}/"
                DESTINATION "${COMMON_INSTALL_DIR}/include"
                FILES_MATCHING PATTERN "*.hpp"
        )
    endif()
else()
    find_package(CycloneDDS-CXX REQUIRED)
    idlcxx_generate(TARGET joy_data_msg
        FILES
        msgs/idl/JoyData.idl
        WARNINGS no-implicit-extensibility
    )

    idlcxx_generate(TARGET cli_data_msg
        FILES
        msgs/idl/CliData.idl
        WARNINGS no-implicit-extensibility
    )

    add_library(quad_communication SHARED
        src/CommunicationManager.cpp
        src/SHM.cpp
    )
    target_link_libraries(quad_communication
        joy_data_msg
        cli_data_msg
        CycloneDDS-CXX::ddscxx
    )
    install(TARGETS quad_communication
        ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
        LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
    )

    file(GLOB ROS2_MSG_FILES msgs/idl/ros2/*.idl)

    if(COMM_TYPE STREQUAL "dds")
        idlcxx_generate(TARGET quad_comm_msgs
        FILES
        msgs/idl/FloatScalar.idl
        msgs/idl/Point.idl
        msgs/idl/JointData.idl
        msgs/idl/SensorData.idl
        msgs/idl/JoyData.idl
        msgs/idl/QuadLog.idl
        msgs/idl/SolverStats.idl
        msgs/idl/CliData.idl
        msgs/idl/IndicatorData.idl
        ${ROS2_MSG_FILES}
        WARNINGS no-implicit-extensibility)

        set(GENERATED_HEADERS_DIR "${CMAKE_CURRENT_BINARY_DIR}")

        # Install headers
        install(DIRECTORY "${GENERATED_HEADERS_DIR}/"
                DESTINATION "${COMMON_INSTALL_DIR}/include"
                FILES_MATCHING PATTERN "*.hpp"
        )

        add_library(dds_comm SHARED
            src/QuadDDSComm.cpp
        )
        target_link_libraries(dds_comm
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
            quad_communication
            Threads::Threads
        )
        install(TARGETS
            dds_comm
            ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
            LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
        )

        add_executable(test_dds_subscriber
            tests/test_dds_subscriber.cpp
        )
        target_link_libraries(test_dds_subscriber
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
        )

        add_executable(sensor_data_test
            tests/sensor_data_test.cpp
        )
        target_link_libraries(sensor_data_test
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
        )

        add_executable(test_pub
            tests/test_dds_pub.cpp
        )
        target_link_libraries(test_pub
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
        )

        add_executable(test_sub
            tests/test_dds_sub.cpp
        )
        target_link_libraries(test_sub
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
        )

        add_executable(test_dds_comm
            tests/test_dds_comm.cpp
        )
        target_link_libraries(test_dds_comm
            dds_comm
            quad_comm_msgs
            CycloneDDS-CXX::ddscxx
        )

        if (NOT APPLE)
            add_executable(joystick_interface_dds
                 src/joystick_interface.cpp
            )
            target_link_libraries(joystick_interface_dds
                 quad_comm_msgs
                 CycloneDDS-CXX::ddscxx
            )

            add_executable(joystick_mux
                src/joystick_mux.cpp
            )
            target_link_libraries(joystick_mux
                quad_communication
                CycloneDDS-CXX::ddscxx
            )

            add_executable(cli_interface_dds
                src/cli_interface.cpp
            )
            target_link_libraries(cli_interface_dds
                quad_comm_msgs
                quad_communication
                dds_comm
                CycloneDDS-CXX::ddscxx
            )

            add_executable(keyboard_interface_dds
                src/keyboard_interface.cpp
            )
            target_include_directories(keyboard_interface_dds PRIVATE
                ${COMMON_INSTALL_DIR}/include
            )
            target_link_libraries(keyboard_interface_dds
                quad_comm_msgs
                CycloneDDS-CXX::ddscxx
                ${COMMON_INSTALL_DIR}/lib/libyaml-cpp.so
            )

            add_executable(p2p_interface_dds
                src/p2p_interface_dds.cpp
            )
            target_include_directories(p2p_interface_dds PRIVATE
                ${COMMON_INSTALL_DIR}/include
            )
            target_link_libraries(p2p_interface_dds
                quad_comm_msgs
                CycloneDDS-CXX::ddscxx
                ${COMMON_INSTALL_DIR}/lib/libyaml-cpp.so
            )

            install(TARGETS
                test_dds_comm
                test_pub
                test_sub
                test_dds_subscriber
                sensor_data_test
                joystick_interface_dds
                joystick_mux
                keyboard_interface_dds
                p2p_interface_dds
                ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
                RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
            )
        endif()
    endif()

    if(COMM_TYPE STREQUAL "ros2")
        add_library(ros_comm SHARED
        src/QuadROSComm.cpp
        )

        target_link_libraries(ros_comm
            ${rclcpp_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
            ${nav_msgs_LIBRARIES}
            ${geometry_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
            quad_communication
            Threads::Threads
        )
        install(TARGETS
            ros_comm
            ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
            LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
        )

        add_executable(test_ros_comm
            tests/test_ros_comm.cpp
        )
        target_link_libraries(test_ros_comm
            ros_comm
        )
        install(TARGETS
            test_ros_comm
            ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
            RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
        )
    endif()

    if(COMM_TYPE STREQUAL "ros")
        add_library(ros1_comm SHARED
        src/QuadROS1Comm.cpp
        )

        target_link_libraries(ros1_comm
            ${roscpp_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
            ${nav_msgs_LIBRARIES}
            ${geometry_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
            quad_communication
            Threads::Threads
        )
        install(TARGETS
            ros1_comm
            ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
            LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
        )
    endif()
endif()
