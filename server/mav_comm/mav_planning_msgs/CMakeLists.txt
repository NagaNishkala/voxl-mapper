set(TARGET mav_planning_msgs)

add_library(${TARGET} INTERFACE)

target_include_directories(${TARGET} INTERFACE include/ /usr/include/voxblox/Eigen)

install(
  DIRECTORY include/
  DESTINATION /usr/include/${TARGET}/
  FILES_MATCHING PATTERN "*.h"
)
