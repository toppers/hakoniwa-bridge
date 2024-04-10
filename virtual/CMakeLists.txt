cmake_minimum_required(VERSION 3.10)

project(VirtualProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(THIRD_PARTY_PATH "${CMAKE_SOURCE_DIR}/../third-party")
set(HAKO_INC_DIR "${THIRD_PARTY_PATH}/hakoniwa-core-cpp-client/src/include")
set(CMAKE_FIND_ROOT_PATH /usr/local/lib)
if (DEFINED HAKO_CLIENT_OPTION_FILEPATH)
    include(${HAKO_CLIENT_OPTION_FILEPATH})
else()
    include(default-cmake-options.cmake)
endif()

find_library(ZENOH_C_LIBRARY NAMES zenohc NO_DEFAULT_PATH HINTS ${CMAKE_FIND_ROOT_PATH})
find_library(HAKO_C_LIBRARY NAMES assets NO_DEFAULT_PATH HINTS ${CMAKE_FIND_ROOT_PATH}/hakoniwa)
find_library(HAKO_CONDUCTOR_C_LIBRARY NAMES conductor NO_DEFAULT_PATH HINTS ${CMAKE_FIND_ROOT_PATH}/hakoniwa)

message(STATUS "ZENOH_C_LIBRARY_PATH: " ${ZENOH_C_LIBRARY})
message(STATUS "HAKO_C_LIBRARY: " ${HAKO_C_LIBRARY})
message(STATUS "HAKO_CONDUCTOR_C_LIBRARY: " ${HAKO_CONDUCTOR_C_LIBRARY})

add_subdirectory(shm-proxy)
