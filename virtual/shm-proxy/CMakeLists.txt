cmake_minimum_required(VERSION 3.10)

project(shm-proxy-project)

add_executable(
    shm-proxy 
    src/shm_proxy.cpp
    src/shm_proxy_pdu.cpp
    src/zenoh_comm.cpp
)

target_include_directories(
    shm-proxy
    PRIVATE /usr/local/include
    PRIVATE ${HAKO_INC_DIR}
)

target_link_libraries(
    shm-proxy
    ${ZENOH_C_LIBRARY}
    ${HAKO_C_LIBRARY}
    ${HAKO_CONDUCTOR_C_LIBRARY}
)
