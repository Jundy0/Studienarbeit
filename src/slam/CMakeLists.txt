cmake_minimum_required(VERSION 3.5)

project(Studienarbeit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include("${CMAKE_CURRENT_SOURCE_DIR}/../core/coreInclude.cmake")

add_executable(main main.cpp ${CORE_CLASS_FILES})
