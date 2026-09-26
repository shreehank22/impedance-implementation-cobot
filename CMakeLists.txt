cmake_minimum_required(VERSION 3.16)
project(RIMaR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_BUILD_TYPE Debug)

# Third-party library paths
set(COMMON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/custom/include)
set(COMMON_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/custom/lib)
set(COMMON_INSTALL_DIR ${CMAKE_SOURCE_DIR}/custom/install)

include_directories(
    ${COMMON_INCLUDE_DIR}
    ${COMMON_INSTALL_DIR}/include
)

list(APPEND CMAKE_PREFIX_PATH ${COMMON_INSTALL_DIR})
list(APPEND CMAKE_INSTALL_RPATH ${COMMON_INSTALL_DIR}/lib)
list(APPEND CMAKE_INSTALL_RPATH ${COMMON_LIBRARY_DIR})

# option(USE_HARDWARE "Using real hardware instead of simulation" ON)
set(USE_HARDWARE OFF CACHE BOOL "Using real hardware instead of simulation" FORCE)

# message(STATUS "USE_HARDWARE is set to: ${USE_HARDWARE}")

add_compile_definitions($<$<BOOL:${USE_HARDWARE}>:USE_HARDWARE>)

# Add subdirectories
add_subdirectory(src/utils)
add_subdirectory(src/common)
add_subdirectory(src/dynamics)
add_subdirectory(src/communication)
add_subdirectory(src/execution)
if (${USE_HARDWARE})
    add_subdirectory(src/hardware)
else()
    add_subdirectory(src/robot_simulation)
endif()