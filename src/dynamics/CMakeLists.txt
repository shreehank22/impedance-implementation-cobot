# --- Dependencies Setup ---
set(CMAKE_PREFIX_PATH ${COMMON_INSTALL_DIR} ${CMAKE_PREFIX_PATH})

find_package(Eigen3 REQUIRED)
# Re-enabled Pinocchio
find_package(pinocchio REQUIRED)

message("eigen include: ${eigen3_INCLUDE_DIRS}")
message("pinocchio include: ${pinocchio_INCLUDE_DIRS}")
# Silence legacy hpp-fcl warnings from coal / pinocchio
add_compile_definitions(COAL_DISABLE_HPP_FCL_WARNINGS)
# --- Include Paths ---
include_directories(
    include
    ${COMMON_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/include
    ${pinocchio_INCLUDE_DIRS})
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(execution LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(iir REQUIRED)
find_package(Threads REQUIRED)
# find_package(qpOASES REQUIRED)
find_package(yaml-cpp REQUIRED)

# Fallback paths for standalone builds from src/execution (cmake ..)
if(NOT DEFINED COMMON_INCLUDE_DIR)
    set(COMMON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../custom/include")
endif()
if(NOT DEFINED COMMON_LIBRARY_DIR)
    set(COMMON_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../custom/lib")
endif()
if(NOT DEFINED COMMON_INSTALL_DIR)
    set(COMMON_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../custom/install")
endif()

if(DEFINED ENV{USE_COMM_TYPE})
    if($ENV{USE_COMM_TYPE} STREQUAL "ros")
        find_package(roscpp REQUIRED)
        include_directories(
            ${roscpp_INCLUDE_DIRS}
        )
    endif()
endif()

include_directories(
    include
    ${COMMON_INCLUDE_DIR}
    ${COMMON_INSTALL_DIR}/include
    ${COMMON_INSTALL_DIR}/include/eigen3
    ${COMMON_INSTALL_DIR}/include/ddscxx
    ${CMAKE_CURRENT_SOURCE_DIR}/../dynamics/include
    # ${qpOASES_INCLUDE_DIR}
)
link_directories(
    ${COMMON_LIBRARY_DIR}
    ${COMMON_INSTALL_DIR}/lib
)

# Some environments expose only `yaml-cpp` (not imported target). Provide fallback.
if(NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
    set_property(TARGET yaml-cpp::yaml-cpp PROPERTY INTERFACE_LINK_LIBRARIES yaml-cpp)
endif()

if(INSTALL_HEADERS_ONLY)
    install(DIRECTORY include/
        DESTINATION ${COMMON_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h*"
    )
else()
    # --- Build Library ---
    add_library(quad_dynamics SHARED
        src/kinodynamics.cpp
    )
    
    # CRITICAL: Linking pinocchio::pinocchio is what fixes the "urdf.hpp" error
    target_link_libraries(quad_dynamics
        pinocchio::pinocchio
        stdc++fs
    )

    install(TARGETS quad_dynamics
        ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
        LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
    )

    # --- Build Executable ---
    add_executable(test_kinodynamics
        tests/test_kinodynamics.cpp
    )
    
    target_link_libraries(test_kinodynamics
        quad_dynamics
    )
    target_compile_definitions(quad_dynamics PRIVATE
    COAL_DISABLE_HPP_FCL_WARNINGS
    )

    install(TARGETS 
        test_kinodynamics
        ARCHIVE DESTINATION ${COMMON_INSTALL_DIR}
        RUNTIME DESTINATION ${COMMON_INSTALL_DIR}
    )
endif()
