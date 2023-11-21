cmake_minimum_required(VERSION 3.0)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src"
)

aux_source_directory(
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src"
    RPLIDAR_SDK_SRC
)

# Your source files
add_executable(test test.cpp ${RPLIDAR_SDK_SRC}
)

# Link against the RPLIDAR SDK libraries
# target_link_libraries(test rplidar)
