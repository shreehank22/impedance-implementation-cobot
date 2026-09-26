if(INSTALL_HEADERS_ONLY)
install(DIRECTORY include/
    DESTINATION ${COMMON_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h*"
)
else()
include_directories(
    include
)
add_library(lp_filter SHARED
    src/Filter.cpp
    src/LowPassFilter.cpp
)
add_library(ma_filter SHARED
    src/Filter.cpp
    src/MovingAverageFilter.cpp
)

find_package(iir REQUIRED)
add_library(iir_filter SHARED
    src/Filter.cpp
    src/IIRFilter.cpp
)
target_link_libraries(iir_filter
    ${COMMON_LIBRARIES}
    iir::iir
)

add_library(acc_limit_filter SHARED
    src/Filter.cpp
    src/AccLimitFilter.cpp
)
target_link_libraries(acc_limit_filter
    ${COMMON_LIBRARIES}
)

install(TARGETS
    lp_filter
    ma_filter
    iir_filter
    acc_limit_filter
    ARCHIVE DESTINATION ${COMMON_LIBRARY_DIR}
    LIBRARY DESTINATION ${COMMON_LIBRARY_DIR}
)

endif()