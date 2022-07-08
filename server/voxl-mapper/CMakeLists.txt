set(TARGET voxl-mapper)

file(GLOB_RECURSE all_src_files "*.c" "*.cc")
add_executable(${TARGET} ${all_src_files})

target_include_directories(${TARGET} PUBLIC
    ./
    /usr/include/voxl_trajectory
)

target_link_libraries(${TARGET}
    pthread
    /usr/lib64/libvoxblox.so
    /usr/lib64/libmodal_pipe.so
    /usr/lib64/libmodal_json.so
    /usr/lib64/libvoxl_cutils.so
    /usr/lib64/libnlopt.so
    /usr/lib64/libvoxl_common_config.so
    mav_msgs
    mav_planning_msgs
    mav_trajectory_generation
    mav_planning_common
    mav_path_smoothing
    mav_local_planner
)

install(
	TARGETS ${TARGET}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
