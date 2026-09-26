set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)

find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

include_directories(
    ${COMMON_INSTALL_DIR}/include
    include
)
link_directories(${COMMON_INSTALL_DIR}/lib)

if(INSTALL_HEADERS_ONLY)
    install(DIRECTORY include/
        DESTINATION ${COMMON_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
    )

else()
    # ── lodepng (screenshot support used by simulate.cc) ─────────────────────
    add_library(lodepng STATIC src/lodepng.cpp include/lodepng.h)
    target_include_directories(lodepng PUBLIC include)

    # ── GLFW platform adapter (abstract PlatformUIAdapter + GlfwAdapter) ─────
    add_library(platform_ui_adapter OBJECT)
    target_sources(platform_ui_adapter
        PUBLIC  include/glfw_adapter.h
                include/glfw_dispatch.h
                include/platform_ui_adapter.h
        PRIVATE src/glfw_adapter.cc
                src/glfw_dispatch.cc
                src/platform_ui_adapter.cc
    )
    target_include_directories(platform_ui_adapter PUBLIC
        ${COMMON_INCLUDE_DIR}
        $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
    )
    target_link_libraries(platform_ui_adapter PUBLIC mujoco)

    # ── LAIR Simulate library (full dual-panel mjUI, profiler, sensors) ──────
    add_library(simulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
    target_sources(simulate
        PUBLIC  include/simulate.h
                include/array_safety.h
        PRIVATE src/simulate.cc
    )
    target_include_directories(simulate PUBLIC include ${COMMON_INCLUDE_DIR})
    target_link_libraries(simulate PUBLIC lodepng platform_ui_adapter mujoco)

    # ── simulate_pv — LAIR-identical UI, DDS-driven cobot controller ─────────
    add_executable(simulate_pv src/simulate_pv.cpp)
    target_include_directories(simulate_pv PRIVATE
        include
        ${COMMON_INSTALL_DIR}/include
        ${COMMON_INSTALL_DIR}/include/ddscxx
    )
    target_link_libraries(simulate_pv
        simulate
        mujoco
        glfw
        Threads::Threads
        dds_comm
        CycloneDDS-CXX::ddscxx
    )
    # COMMON_INSTALL_DIR = custom/install  (mujoco, glfw, ...)
    # custom/lib                           (dds_comm, quad_communication, ...)
    set(_rpath "${COMMON_INSTALL_DIR}/lib:${COMMON_INSTALL_DIR}/lib64:${CMAKE_SOURCE_DIR}/custom/lib")
    set_target_properties(simulate_pv PROPERTIES
        BUILD_RPATH              "${_rpath}"
        INSTALL_RPATH            "${_rpath}"
        INSTALL_RPATH_USE_LINK_PATH TRUE
        BUILD_WITH_INSTALL_RPATH    TRUE
    )

    install(TARGETS simulate_pv RUNTIME DESTINATION ${COMMON_INSTALL_DIR})

    install(DIRECTORY include/
        DESTINATION ${COMMON_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
    )
endif()
