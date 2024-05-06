set(HAKO_CMAKE_VERSION ${HAKO_CMAKE_VERSION})

project(hako-bridge-sample
    LANGUAGES C CXX
)


message(STATUS "CMAKE_VERSION:${CMAKE_VERSION}" )
link_directories(/usr/local/lib/hakoniwa)

set(OS_TYPE "posix")
if(WIN32)
    set(OS_TYPE "win")
endif(WIN32)
MESSAGE(STATUS "OS_TYPE=" ${OS_TYPE})

add_executable(
    sample
    sample.c
)

target_include_directories(
    sample
    PRIVATE include
    PRIVATE /usr/local/include
    PRIVATE ../third-party/hakoniwa-core-cpp-client/src/include
)

target_link_libraries(
    sample
    assets
    conductor
)
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(
        sample
        Threads::Threads
    )
endif()
