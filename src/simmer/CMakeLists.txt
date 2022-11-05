
cmake_minimum_required(VERSION 3.18)

project(
    "simmer"
    VERSION 0.1.0
    DESCRIPTION
    "C++ library | Egress Core Simulation Engine"
    LANGUAGES CXX)

set(TARGET_NAME "simmer")

file(GLOB_RECURSE horce_files CONFIGURE_DEPENDS
    "[^.]*\.hpp"
    "[^.]*\.cpp"
    )

add_library(${TARGET_NAME}
    ${horce_files}
    )

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported)
option(USE_IPO "Use IPO" ${ipo_supported})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set_property(TARGET ${TARGET_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
endif()

target_compile_options(${TARGET_NAME}
    PRIVATE ${COMMON_COMPILE_OPTIONS}
    )

set_target_properties(${TARGET_NAME} PROPERTIES
    CXX_STANDARD          20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS        ON
    )

add_subdirectory(share)

add_library(fs INTERFACE)

target_link_libraries(${TARGET_NAME}
    PRIVATE CDT
    PRIVATE fmt
    PRIVATE fs
    PRIVATE pugixml
    )

target_include_directories(${TARGET_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src
    )

