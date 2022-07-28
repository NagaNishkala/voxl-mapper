set(TARGET mav_local_planner)

file(GLOB all_src_files  "src/*.cpp")
add_library(${TARGET} SHARED ${all_src_files})

target_include_directories(${TARGET} PUBLIC
    include/
)

target_link_libraries(${TARGET}
    mav_planning_msgs
    mav_path_smoothing
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
