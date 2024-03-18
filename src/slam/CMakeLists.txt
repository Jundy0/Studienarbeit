cmake_minimum_required(VERSION 3.5)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(CLASS_FILES
	"${CMAKE_CURRENT_SOURCE_DIR}/src/occupancy_grid.cpp"
)

# Add the main executable
add_executable(main main.cpp ${CLASS_FILES})
