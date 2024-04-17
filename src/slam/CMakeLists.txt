cmake_minimum_required(VERSION 3.5)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CLASS_FILES
	"${CMAKE_CURRENT_SOURCE_DIR}/src/occupancy_grid.cpp"
)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen/Eigen"
)

# Add the main executable
add_executable(main main.cpp ${CLASS_FILES})
