set(TARGET voxl-mapper)

set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra -Wuninitialized -Wattributes -Wno-unused-parameter \
	-Wno-sign-compare -Wunused-but-set-variable -Wdouble-promotion -Wno-missing-field-initializers \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

file(GLOB_RECURSE all_src_files "*.c" "*.cc")
add_executable(${TARGET} ${all_src_files})

target_include_directories(${TARGET} PUBLIC
    ./
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
    gomp
)

install(
	TARGETS ${TARGET}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
