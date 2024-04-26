cmake_minimum_required(VERSION 3.0)

project(Studienarbeit_Simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(CLASS_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lidarSensorSim.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vehicleActuatorSim.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/simulationManager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/obstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/intersection.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/controlWindow.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/visualizeWindow.cpp"
)

find_package(SFML 2 COMPONENTS graphics window system REQUIRED)

include_directories(${SFML_INCLUDE_DIR})

include("${CMAKE_CURRENT_SOURCE_DIR}/../core/coreInclude.cmake")

add_executable(main main.cpp ${CLASS_FILES} ${CORE_CLASS_FILES} ${SLAM_CLASS_FILES})

target_link_libraries(main PRIVATE sfml-graphics sfml-window sfml-system)
