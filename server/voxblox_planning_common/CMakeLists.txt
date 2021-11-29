set(TARGET voxblox_planning_common)
file(GLOB all_src_files  "src/*.cpp")
add_library(${TARGET} SHARED ${all_src_files})

target_include_directories(${TARGET} PUBLIC
    include/
)

target_link_libraries(${TARGET}
    mav_msgs
    mav_planning_common
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
