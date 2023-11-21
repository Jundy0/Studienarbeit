cmake_minimum_required(VERSION 3.0)
project(YourProjectName)

set(CMAKE_CXX_STANDARD 11)

# Include directories
include_directories(/lib/rplidar_sdk/sdk/include)

# Library directories
link_directories(/lib/rplidar_sdk/sdk/src)

# Your source files
add_executable(YourExecutable test.cpp)

# Link against the RPLIDAR SDK libraries
target_link_libraries(YourExecutable rplidar)
