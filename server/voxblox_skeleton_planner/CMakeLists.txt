set(TARGET voxblox_skeleton_planner)

set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra -Wuninitialized -Wattributes -Wno-unused-parameter \
	-Wno-sign-compare -Wunused-but-set-variable -Wdouble-promotion -Wno-missing-field-initializers \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

file(GLOB all_src_files  "src/*.cpp")
add_library(${TARGET} SHARED ${all_src_files})

target_include_directories(${TARGET} PUBLIC
    include/
    /usr/include/voxblox/Eigen
)

target_link_libraries(${TARGET}
    mav_msgs
    mav_planning_msgs
    mav_trajectory_generation
    mav_planning_common
    mav_path_smoothing
    voxblox_skeleton
    voxblox_planning_common
)

include_directories(${TARGET}
    /usr/include/voxblox/
    /usr/include/
)

link_directories(
    /usr/lib64/
)

target_link_libraries(${TARGET}
    pthread
    /usr/lib64/libvoxblox.so
    /usr/lib64/libnlopt.so
)

install(
    TARGETS ${TARGET}
    LIBRARY			DESTINATION /usr/lib64/
    RUNTIME			DESTINATION /usr/bin
    PUBLIC_HEADER	DESTINATION /usr/include/${TARGET}/
)

install(
  DIRECTORY include/
  DESTINATION /usr/include/${TARGET}/
  FILES_MATCHING PATTERN "*.h"
)
