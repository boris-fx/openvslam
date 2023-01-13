# ----- Find dependencies -----

# Pangolin
#find_package(Pangolin REQUIRED)

# ----- Configure PangolinViewer library -----

add_library(pangolin_viewer
            ${CMAKE_CURRENT_SOURCE_DIR}/viewer.h
            ${CMAKE_CURRENT_SOURCE_DIR}/color_scheme.h
            ${CMAKE_CURRENT_SOURCE_DIR}/viewer.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/color_scheme.cc
			${CMAKE_CURRENT_SOURCE_DIR}/exports.h)

add_compile_definitions(PANGOLIN_VIEWER_DLL_EXPORT)
add_compile_definitions(HAVE_GLEW)
add_compile_definitions(USE_EIGEN)

set_target_properties(pangolin_viewer PROPERTIES
                      OUTPUT_NAME pangolin_viewer
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
if(${Pangolin_VERSION} VERSION_GREATER 0.5)
    message(STATUS "Build pangolin_viewer with c++17")
    set_property(TARGET pangolin_viewer PROPERTY CXX_STANDARD 17)
endif()

#message("Pangolin_INCLUDE_DIR: ${Pangolin_INCLUDE_DIR}")

target_include_directories(pangolin_viewer 
                           PUBLIC
 						   "${Pangolin_INCLUDE_DIR}"
						   "${Pangolin_INCLUDE_DIR_CORE}"
							"${Pangolin_INCLUDE_DIR_DISPLAY}"
							"${Pangolin_INCLUDE_DIR_OPENGL}"
							"${Pangolin_INCLUDE_DIR_WINDOWING}"
							"${Pangolin_INCLUDE_DIR_VARS}"
							"${Pangolin_INCLUDE_DIR_IMAGE}"
							"${Pangolin_INCLUDE_DIR_VIDEO}"
							"${Pangolin_INCLUDE_DIR_PLOT}"
							"${GLEW_INCLUDE_DIR}")

message("******************========>>>>>>>>>> Pangolin_LIBS: ${PROJECT_NAME}")

target_link_libraries(pangolin_viewer
                      PUBLIC
                      ${PROJECT_NAME}
                      opencv_highgui
					  "${Pangolin_LIBRARIES}"
						"${Pangolin_LIB_CORE}"
						"${Pangolin_LIB_DISPLAY}"
						"${Pangolin_LIB_OPENGL}"
						"${Pangolin_LIB_WINDOWING}"
						"${Pangolin_LIB_VARS}"
						"${Pangolin_LIB_IMAGE}"
						"${Pangolin_LIB_VIDEO}"
						"${Pangolin_LIB_PLOT}"
						"${GLEW_LIB}"
						"${OPENGL_LIB}")

# ----- Install configuration -----

if(INSTALL_PANGOLIN_VIEWER)
    set(PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/pangolin_viewer)

    install(TARGETS pangolin_viewer
            EXPORT ${STELLA_VSLAM_TARGETS_EXPORT_NAME}
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})

    file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    install(FILES ${HEADERS}
            DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})
endif()
