set (SLAM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../slam/include")
set (SLAM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../slam/src")

include_directories(
    "${SLAM_INCLUDE_DIR}"
)

set(CORE_CLASS_FILES
    "${SLAM_SOURCE_DIR}/ekf.cpp"
)
