set(TARGET voxblox_skeleton)
file(GLOB all_src_files  "src/*.cpp")
add_library(${TARGET} SHARED ${all_src_files})

target_include_directories(${TARGET} PUBLIC
    include/
    /usr/include/voxblox/Eigen
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
