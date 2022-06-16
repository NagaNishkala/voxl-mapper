set(TARGET voxblox_loco_planner)
file(GLOB all_src_files  "src/*.cpp")
add_library(${TARGET} SHARED ${all_src_files})

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
