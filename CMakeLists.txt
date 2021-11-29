cmake_minimum_required(VERSION 3.3)
project(voxl-mapper)

set (ROOT ./server)

# James' standard list of cmake flags
set(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wno-double-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=gnu++14 -Wall -Wextra -Wuninitialized -Wattributes \
	-Wunused-but-set-variable -Wdouble-promotion -Wno-missing-field-initializers \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
	set(LIB_INSTALL_DIR /usr/lib64)
else()
	set(LIB_INSTALL_DIR /usr/lib)
endif()

add_subdirectory(server/mav_comm/mav_msgs)
add_subdirectory(server/mav_comm/mav_planning_msgs)
add_subdirectory(server/mav_trajectory_generation)
add_subdirectory(server/mav_planning_common)
add_subdirectory(server/loco_planner)
add_subdirectory(server/voxblox_planning_common)
add_subdirectory(server/voxblox_loco_planner)
add_subdirectory(server/mav_path_smoothing)
add_subdirectory(server/voxl-mapper)
add_subdirectory(server/mav_local_planner)
