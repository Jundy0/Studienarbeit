set (SLAM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../slam/include")
set (SLAM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../slam/src")
set (SLAM_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../slam/lib")

include_directories(
	"${SLAM_INCLUDE_DIR}"
	"${SLAM_LIBRARY_DIR}"
	"${SLAM_LIBRARY_DIR}/eigen"
)

set(SLAM_CLASS_FILES
	"${SLAM_SOURCE_DIR}/icpHandler.cpp"
	"${SLAM_SOURCE_DIR}/occupancyGrid.cpp"
	"${SLAM_SOURCE_DIR}/particle.cpp"
	"${SLAM_LIBRARY_DIR}/icp/icp.cpp"
)
