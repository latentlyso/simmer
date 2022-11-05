
################################################################################
# pugixml
################################################################################

add_library(pugixml
    pugixml/pugixml.cpp
    pugixml/pugixml.hpp
    pugixml/pugiconfig.hpp
    )

target_include_directories(${TARGET_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/pugixml
    )

################################################################################
# CDT
################################################################################

add_subdirectory(CDT)

target_include_directories(${TARGET_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/CDT/include
    )

################################################################################
# fmt
################################################################################

add_subdirectory(fmt)

target_include_directories(${TARGET_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/fmt/include
    )
    

