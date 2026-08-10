cmake_minimum_required(VERSION 3.10)
project(SensorProject)

set(CMAKE_CXX_STANDARD 17)

# 내 프로젝트의 모든 소스 파일을 여기에 다 적어줍니다!
add_executable(SensorTest SensorTest.cpp IMU.cpp Lidar.cpp)

target_include_directories(SensorTest
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils
)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(SensorTest PRIVATE
            $<$<CONFIG:Debug>:-fsanitize=address>
            $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    )
    target_link_options(SensorTest PRIVATE
            $<$<CONFIG:Debug>:-fsanitize=address>
    )
endif()