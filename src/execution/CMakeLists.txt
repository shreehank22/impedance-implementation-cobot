find_package(iir REQUIRED)
find_package(Threads REQUIRED)
# find_package(qpOASES REQUIRED)
find_package(yaml-cpp REQUIRED)


if("$ENV{USE_COMM_TYPE}" STREQUAL "ros")
    find_package(roscpp REQUIRED)
    include_directories(
        ${roscpp_INCLUDE_DIRS}
    )
endif()

include_directories(
    include
    ${COMMON_INCLUDE_DIR}
    # ${qpOASES_INCLUDE_DIR}
)

if(INSTALL_HEADERS_ONLY)
set(EFFECTIVE_COMM_TYPE "$ENV{USE_COMM_TYPE}")
if(NOT EFFECTIVE_COMM_TYPE)
    set(EFFECTIVE_COMM_TYPE "dds")
endif()
if(EFFECTIVE_COMM_TYPE STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif(EFFECTIVE_COMM_TYPE STREQUAL "ros2")
    add_definitions(-DUSE_ROS2_COMM)
elseif(EFFECTIVE_COMM_TYPE STREQUAL "ros")
    add_definitions(-DUSE_ROS_COMM)
endif()
install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)
else()

add_library(pd_controller STATIC
    src/Controller/Controller.cpp
)
target_compile_options(pd_controller PRIVATE "-fPIC")
target_link_libraries(pd_controller
    quad_dynamics
    iir::iir
    # ${qpOASES_LIBRARY}
    yaml-cpp::yaml-cpp
)


add_library(quad_estimator SHARED
    src/Estimator/Estimator.cpp
    # src/Estimator/EKFEstimator.cpp
)
target_link_libraries(quad_estimator
    quad_dynamics
    iir::iir
    lp_filter
    ma_filter
    iir_filter
    acc_limit_filter
    yaml-cpp::yaml-cpp
)

add_library(base_planner SHARED
    src/Planner/Planner.cpp
    # src/Planner/Gait.cpp
    # src/Planner/FootSwingTrajectory.cpp
)
target_link_libraries(base_planner
    quad_dynamics
    yaml-cpp::yaml-cpp
)




install(TARGETS 
    pd_controller
    # didc
    # bc
    # cnvxmpc
    # mrt_controller
    quad_estimator
    base_planner
    # reactive_planner
    # mc_planner
    ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
    LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

file(GLOB_RECURSE FSM_SRC_LIST
    "src/StateMachine/*.cpp"
    "src/StateMachine/*.cc"
)

add_executable(run src/run.cpp ${FSM_SRC_LIST})



# add_executable(Estimator_test tests/Estimator_test.cpp)
# target_link_libraries(Estimator_test 
#     quad_estimator
#     quad_dynamics
#     dds_comm
# )

# add_executable(Controller_test tests/Controller_test.cpp)
# target_link_libraries(Controller_test 
#     quad_estimator
#     quad_dynamics
#     dds_comm
#     base_planner
#     pd_controller
# )



set(EFFECTIVE_COMM_TYPE "$ENV{USE_COMM_TYPE}")
if(NOT EFFECTIVE_COMM_TYPE)
    set(EFFECTIVE_COMM_TYPE "dds")
endif()
if(EFFECTIVE_COMM_TYPE STREQUAL "dds")
    add_definitions(-DUSE_DDS_COMM)
elseif(EFFECTIVE_COMM_TYPE STREQUAL "ros2")
    add_definitions(-DUSE_ROS2_COMM)
elseif(EFFECTIVE_COMM_TYPE STREQUAL "ros")
    add_definitions(-DUSE_ROS_COMM)
endif()

# add_dependencies(run
#     quad_communication
#     # didc
#     # bc
#     # mrt_controller
#     quad_dynamics
#     quad_estimator
#     base_planner
#     # reactive_planner
#     # mc_planner
# )
target_link_libraries(run
    pd_controller
    # didc
    # bc
    # mrt_controller
    quad_dynamics
    quad_estimator
    base_planner
    # reactive_planner
    # mc_planner
    Threads::Threads
    quad_communication
)

if(EFFECTIVE_COMM_TYPE STREQUAL "dds")
    target_link_libraries(run
        dds_comm
    )
elseif(EFFECTIVE_COMM_TYPE STREQUAL "ros2")
    target_link_libraries(run
        ros_comm
    )
elseif(EFFECTIVE_COMM_TYPE STREQUAL "ros")
    target_link_libraries(run
        ros1_comm
    )
endif()

# add_executable(test_cnvxmpc 
#     tests/test_cnvxmpc.cpp
#     src/Planner/Gait.cpp
# )
# target_link_libraries(test_cnvxmpc
#     cnvxmpc
#     base_planner
# )

# add_executable(test_mpc 
#     tests/example_hpipm_mpc.cpp
# )
# target_link_libraries(test_mpc
#     hpipm-cpp::hpipm-cpp
# )

install(TARGETS 
    run
    # Estimator_test
    # Planner_test
    # Controller_test
    # test_cnvxmpc
    # test_mpc
    ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
    RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
)

endif()