aux_source_directory( . AUX_SRC_SLAM_UTILITY_VISUALIZE_3D )

# Add some components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility/src )
if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ${SLAM_UTILITY_PATH}/datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()

# Add all components of slam utility data struct.
if ( NOT TARGET lib_3d_gaussian )
    add_subdirectory( ${SLAM_UTILITY_PATH}/data_struct/3d_gaussian ${PROJECT_SOURCE_DIR}/build/lib_3d_gaussian )
endif()

# Add image painter.
set( IMAGE_PAINTER_PATH ${PROJECT_SOURCE_DIR}/../Image_Painter/src )
if ( NOT TARGET lib_image_painter )
    add_subdirectory( ${IMAGE_PAINTER_PATH} ${PROJECT_SOURCE_DIR}/build/lib_image_painter )
endif()

# Add dependence for visualizor.
if ( NOT TARGET lib_visualize_base_glad_khr )
    add_subdirectory( glad+khr ${PROJECT_SOURCE_DIR}/build/lib_visualize_base_glad_khr )
endif()

add_library( lib_3d_visualizor ${AUX_SRC_SLAM_UTILITY_VISUALIZE_3D} )
target_include_directories( lib_3d_visualizor PUBLIC
    .
    glad+khr
)
target_link_libraries( lib_3d_visualizor
    lib_slam_utility_datatype
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_math
    lib_slam_utility_operate

    lib_3d_gaussian
    lib_image_painter
    lib_visualize_base_glad_khr
)

# Check compile environment. If environment is windows, add additional dependence.
if ( "${COMPILE_ENV}" STREQUAL "windows" )
    # If environment is windows, add special dependence.
    target_include_directories( lib_3d_visualizor PUBLIC
        "E:/glfw-3.3.8.bin.WIN64/include"
    )
    target_link_libraries( lib_3d_visualizor
        "E:/glfw-3.3.8.bin.WIN64/lib-mingw-w64/glfw3.dll"
    )
else()
    # If environment is linux, add special dependence.
    target_link_libraries( lib_3d_visualizor
        -lglfw
    )
endif()
