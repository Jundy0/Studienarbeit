cmake_minimum_required(VERSION 3.0)

project(Studienarbeit_Simulation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(CLASS_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lidarSensorSim.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vehicleActuatorSim.cpp"
)

include("${CMAKE_CURRENT_SOURCE_DIR}/../core/CMakeLists.txt")

add_executable(main main.cpp ${CLASS_FILES} ${CORE_CLASS_FILES})
