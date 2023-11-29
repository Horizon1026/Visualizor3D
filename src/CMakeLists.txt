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
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()

# Add image visualizor.
set( IMAGE_VISUALIZOR_PATH ${PROJECT_SOURCE_DIR}/../Visualizor2D )
if ( NOT TARGET lib_image_visualizor )
    add_subdirectory( ${IMAGE_VISUALIZOR_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_image_visualizor )
endif()

add_library( lib_3d_visualizor ${AUX_SRC_SLAM_UTILITY_VISUALIZE_3D} )
target_include_directories( lib_image_visualizor PUBLIC . )
target_link_libraries( lib_3d_visualizor
    lib_slam_utility_datatype
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_operate

    lib_image_visualizor
)
