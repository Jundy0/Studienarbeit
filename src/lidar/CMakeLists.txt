cmake_minimum_required(VERSION 3.0)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src"
)

set(SDK_FILES
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/rplidar_driver.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_async_transceiver.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_crc.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_lidar_driver.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_lidarprotocol_codec.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_serial_channel.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_tcp_channel.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/sl_udp_channel.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/arch/linux/net_serial.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/arch/linux/net_socket.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/arch/linux/timer.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/dataunpacker/dataunpacker.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_capsules.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_hqnode.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_normalnode.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/sdk/src/hal/thread.cpp"
)

set(CLASS_FILES
"${CMAKE_CURRENT_SOURCE_DIR}/src/a1lidar.cpp"
)

include("${CMAKE_CURRENT_SOURCE_DIR}/../core/CMakeLists.txt")

add_executable(main main.cpp ${CLASS_FILES} ${SDK_FILES} ${CORE_CLASS_FILES})

target_link_libraries(main -pthread pigpio)
