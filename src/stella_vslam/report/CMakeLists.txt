# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/metrics.h
               ${CMAKE_CURRENT_SOURCE_DIR}/plot_html.h
               ${CMAKE_CURRENT_SOURCE_DIR}/debug_printer.h
               ${CMAKE_CURRENT_SOURCE_DIR}/initialisation_debugging.h
               ${CMAKE_CURRENT_SOURCE_DIR}/metrics.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/plot_html.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/debug_printer.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/initialisation_debugging.cc)

if(BUILD_EXAMPLES)
    target_sources(${PROJECT_NAME} PUBLIC ${YAMLCPP_INCLUDE_DIRS})
endif()

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/report)
