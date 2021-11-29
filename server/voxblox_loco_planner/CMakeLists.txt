set(TARGET voxblox_loco_planner)
file(GLOB all_src_files  "src/*.cpp")
add_library(${TARGET} SHARED ${all_src_files})

set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra -Wuninitialized -Wattributes -Wno-unused-parameter \
	-Wno-sign-compare -Wunused-but-set-variable -Wdouble-promotion -Wno-missing-field-initializers \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

target_include_directories(${TARGET} PUBLIC
    include/
)

target_link_libraries(${TARGET}
    loco_planner
    voxblox_planning_common
)

target_link_libraries(${TARGET}
    /usr/lib64/libceres.a
    /usr/lib64/libvoxblox.so
    gomp
)

install(
	TARGETS ${TARGET}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include/${TARGET}/
)

install(
  DIRECTORY include/
  DESTINATION /usr/include/${TARGET}/
  FILES_MATCHING PATTERN "*.h"
)
