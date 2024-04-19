cmake_minimum_required(VERSION 3.5)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CLASS_FILES
	"${CMAKE_CURRENT_SOURCE_DIR}/src/icp_handler.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/load_vectors.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/occupancy_grid.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/icp/icp.cpp"
)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen/Eigen"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/icp"
)

# Add the main executable
add_executable(main main.cpp ${CLASS_FILES})
