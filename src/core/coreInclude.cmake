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
    "${CORE_SOURCE_DIR}/evasionAStar.cpp"
    "${CORE_SOURCE_DIR}/icpHandler.cpp"
	"${CORE_SOURCE_DIR}/occupancyGrid.cpp"
	"${CORE_SOURCE_DIR}/particle.cpp"
    "${CORE_SOURCE_DIR}/slamHandler.cpp"
	"${CORE_LIBRARY_DIR}/icp/icp.cpp"
)
