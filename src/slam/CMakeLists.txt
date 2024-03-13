cmake_minimum_required(VERSION 3.5)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/matplot"
)

# Add the main executable
add_executable(main main.cpp)

# Link against pthread
target_link_libraries(main -pthread matplot)
