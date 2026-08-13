cmake_minimum_required(VERSION 3.16)
project(kimhyunha_c_libs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(c_libs STATIC
    src/MathUtils.cpp
    src/Point3D.cpp
    src/Vector3D.cpp
    src/Volume3D.cpp
)

target_include_directories(c_libs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(CTest)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(test_libs
        test/test_libs.cpp
    )

    target_link_libraries(test_libs PRIVATE c_libs GTest::gtest GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(test_libs)
endif()