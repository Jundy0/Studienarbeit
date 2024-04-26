set (CORE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../core/include")
set (CORE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../core/src")

include("${CMAKE_CURRENT_SOURCE_DIR}/../slam/slamImport.cmake")

include_directories(
    "${CORE_INCLUDE_DIR}"
)

set(CORE_CLASS_FILES
    "${CORE_SOURCE_DIR}/selfdrivingVehicle.cpp"
)
