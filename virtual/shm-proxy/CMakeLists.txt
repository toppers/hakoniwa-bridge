cmake_minimum_required(VERSION 3.15)

project(shm-proxy-project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    shm-proxy 
    src/shm_proxy.cpp
    src/shm_proxy_pdu.cpp
    src/zenoh_comm.cpp
)

if(WIN32)
    target_include_directories(
        shm-proxy
        PRIVATE ${HAKO_INC_DIR}
        PRIVATE ${ZENOH_INC_DIR}
    )
else()
    target_include_directories(
        shm-proxy
        PRIVATE /usr/local/include
        PRIVATE ${HAKO_INC_DIR}
    )
endif()

target_link_libraries(
    shm-proxy
    ${ZENOH_C_LIBRARY}
    ${HAKO_C_LIBRARY}
    ${HAKO_LIBRARY}
    ${HAKO_ASSETS_LIBRARY}
    ${HAKO_CONDUCTOR_C_LIBRARY}
)
