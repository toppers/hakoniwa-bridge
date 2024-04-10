cmake_minimum_required(VERSION 3.10)

project(shm-proxy-project)

add_executable(shm-proxy src/shm-proxy.cpp)

target_include_directories(
    shm-proxy
    PRIVATE /usr/local/include
    PRIVATE ${HAKO_INC_DIR}
)

target_link_libraries(
    shm-proxy
    ${HAKO_C_LIBRARY}
    ${HAKO_CONDUCTOR_C_LIBRARY}
)
