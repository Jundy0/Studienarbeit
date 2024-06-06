set (CORE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../core/include")
set (CORE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../core/src")
set (CORE_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../core/lib")

include_directories(
    "${CORE_INCLUDE_DIR}"
    "${CORE_LIBRARY_DIR}"
    "${CORE_LIBRARY_DIR}/eigen"
)

set(CORE_CLASS_FILES
    "${CORE_SOURCE_DIR}/selfdrivingVehicle.cpp"
    "${CORE_SOURCE_DIR}/evasionControl.cpp"
    "${CORE_SOURCE_DIR}/evasionBasicAStar.cpp"
    "${CORE_SOURCE_DIR}/evasionAdvancedAStar.cpp"
    "${CORE_SOURCE_DIR}/evasionThetaStar.cpp"
    "${CORE_SOURCE_DIR}/occupancyGrid.cpp"
    "${CORE_SOURCE_DIR}/particle.cpp"
    "${CORE_SOURCE_DIR}/slamHandler.cpp"
    "${CORE_SOURCE_DIR}/pclHandler.cpp"
)

find_package(PCL 1.3 REQUIRED COMPONENTS registration common)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
