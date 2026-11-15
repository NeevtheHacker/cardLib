# CMakeLists.txt
# Why: builds and registers the laptop tests for algorithms/ and advanced/ (SPEC section 13),
#      plus header-only hardware/ and chassis/ helpers that never touch PROS.
# Owns: one test executable per *_test.cpp file. Not part of the robot build.
cmake_minimum_required(VERSION 3.20)
project(cardLibTests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CARDLIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(WARNING_FLAGS -Wall -Wextra -Werror)

# Every algorithms/ source, built with the host compiler and nothing from PROS.
file(GLOB_RECURSE ALGORITHM_SOURCES CONFIGURE_DEPENDS
     ${CARDLIB_ROOT}/src/cardLib/algorithms/*.cpp)
add_library(cardLibAlgorithms STATIC ${ALGORITHM_SOURCES})
target_include_directories(cardLibAlgorithms PUBLIC ${CARDLIB_ROOT}/include)
target_compile_options(cardLibAlgorithms PRIVATE ${WARNING_FLAGS})

# Test-only helpers: the drive simulator (sim/) and the advanced/ template, which the robot
# build never compiles.
file(GLOB_RECURSE HELPER_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp
     ${CARDLIB_ROOT}/include/cardLib/advanced/*.cpp)
list(FILTER HELPER_SOURCES EXCLUDE REGEX "_test\\.cpp$")
add_library(cardLibTestHelpers STATIC ${CMAKE_CURRENT_SOURCE_DIR}/doctest_main.cpp
            ${HELPER_SOURCES})
target_include_directories(cardLibTestHelpers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
target_link_libraries(cardLibTestHelpers PUBLIC cardLibAlgorithms)
target_compile_options(cardLibTestHelpers PRIVATE ${WARNING_FLAGS})

enable_testing()
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/*_test.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/advanced/*_test.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/hardware/*_test.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/chassis/*_test.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/sim/*_test.cpp)
foreach(TEST_SOURCE ${TEST_SOURCES})
  get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
  add_executable(${TEST_NAME} ${TEST_SOURCE})
  target_link_libraries(${TEST_NAME} PRIVATE cardLibTestHelpers)
  target_compile_options(${TEST_NAME} PRIVATE ${WARNING_FLAGS})
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
