
set(TARGET_NAME "simmerApp")

add_executable(${TARGET_NAME}
    src/main.cpp
    )

add_library(cxxopts INTERFACE)
add_library(cxxopts::cxxopts ALIAS cxxopts)

target_include_directories(cxxopts INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/src/cxxopts"
    )

target_include_directories(${TARGET_NAME}
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../simmer/share/fmt/include
    )

target_compile_options(${TARGET_NAME} PRIVATE
    ${COMMON_COMPILE_OPTIONS}
    )

target_link_libraries(${TARGET_NAME}
    PRIVATE cxxopts
    PRIVATE fmt
    PRIVATE fs
    PRIVATE simmer
    )

install(TARGETS ${TARGET_NAME}
    DESTINATION bin
    COMPONENT applications)
