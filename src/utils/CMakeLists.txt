# CMakeLists.txt in src/utils
# file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*")

# Install header files
# install(FILES ${HEADER_FILES} DESTINATION ${COMMON_INCLUDE_DIR})
if(INSTALL_HEADERS_ONLY)
install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)
else()
    find_package(yaml-cpp REQUIRED)
    include_directories(
        ${YAML_CPP_INCLUDE_DIRS}
    )
    add_executable(test_config tests/test_config.cpp)
    target_link_libraries(test_config
        yaml-cpp::yaml-cpp
    )

    install(TARGETS
        test_config
        ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
        RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
    )
endif()

# if($ENV{USE_COMM_TYPE} STREQUAL "ros")
#     find_package(roscpp REQUIRED)
#     include_directories(
#         ${roscpp_INCLUDE_DIRS}
#     )
# endif()

include_directories(
    include
    ${COMMON_INSTALL_DIR}/include
)

# if(INSTALL_HEADERS_ONLY)
# if($ENV{USE_COMM_TYPE} STREQUAL "dds")
#     add_definitions(-DUSE_DDS_COMM)
# # elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
# #     add_definitions(-DUSE_ROS2_COMM)
# # elseif($ENV{USE_COMM_TYPE} STREQUAL "ros")
# #     add_definitions(-DUSE_ROS_COMM)
# endif()
#     install(DIRECTORY include/
#         DESTINATION ${COMMON_INCLUDE_DIR}
#         FILES_MATCHING PATTERN "*.h*"
#     )

# else()

# add_executable(joy_interface src/joystick_interface.cpp)




# if($ENV{USE_COMM_TYPE} STREQUAL "dds")
#     add_definitions(-DUSE_DDS_COMM)
# # elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
# #     add_definitions(-DUSE_ROS2_COMM)
# # elseif($ENV{USE_COMM_TYPE} STREQUAL "ros")
# #     add_definitions(-DUSE_ROS_COMM)
# endif()

# target_link_libraries(joy_interface
# quad_communication
# )

# if($ENV{USE_COMM_TYPE} STREQUAL "dds")
#     target_link_libraries(joy_interface
#         dds_comm
#     )
# # elseif($ENV{USE_COMM_TYPE} STREQUAL "ros2")
# #     target_link_libraries(joy_interface
# #         ros_comm
# #     )
# # elseif($ENV{USE_COMM_TYPE} STREQUAL "ros")
# #     target_link_libraries(joy_interface
# #         ros1_comm
# #     )
# endif()

# endif()