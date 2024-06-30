# CMakeのバージョンを設定
cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(HAKO_CMAKE_VERSION ${HAKO_CMAKE_VERSION})

# プロジェクトの設定
project(hako-bridge-sample
    LANGUAGES C CXX
)

# CMakeのバージョンを表示
message(STATUS "CMAKE_VERSION: ${CMAKE_VERSION}" )

# ライブラリのディレクトリをリンク
if(WIN32)
    link_directories(${CMAKE_SOURCE_DIR}/../third-party/local/lib/hakoniwa/)
else()
    link_directories(/usr/local/lib/hakoniwa)
endif()

# OSタイプの設定
set(OS_TYPE "posix")
if(WIN32)
    set(OS_TYPE "win")
endif()
message(STATUS "OS_TYPE= ${OS_TYPE}")

# デバッグ情報を含むビルドタイプを設定
set(CMAKE_BUILD_TYPE Debug)

# Cコンパイラフラグにデバッグ情報を含める
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# 実行ファイルの設定
add_executable(
    sample
    sample.c
)

# インクルードディレクトリの設定
if(WIN32)
    target_include_directories(
        sample
        PRIVATE include
        PRIVATE ../third-party/local/include/hakoniwa
        PRIVATE ../third-party/hakoniwa-ros2pdu/pdu/types
    )
    # リンクするライブラリの設定
    target_link_libraries(
        sample
        assets
        hako
        conductor
    )
else()
    target_include_directories(
        sample
        PRIVATE include
        PRIVATE /usr/local/include
        PRIVATE ../third-party/hakoniwa-core-cpp-client/src/include
        PRIVATE ../third-party/hakoniwa-ros2pdu/pdu/types
    )
    # リンクするライブラリの設定
    target_link_libraries(
        sample
        assets
        conductor
    )
endif()


# POSIX環境でスレッドライブラリをリンク
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(
        sample
        Threads::Threads
    )
endif()
