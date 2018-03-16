cmake_minimum_required(VERSION 3.2)
set(PROJECT_NAME "libstepper")
project(${PROJECT_NAME})
####################################################################################################
####################################################################################################

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Build flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_C_STANDARD_REQUIRED on)
set(DEBUG_BUILD_FLAGS "-Wall -Wextra -Wsign-conversion -pedantic")
set(RELEASE_BUILD_FLAGS "${DEBUG_BUILD_FLAGS} -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${RELEASE_BUILD_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${DEBUG_BUILD_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${RELEASE_BUILD_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${DEBUG_BUILD_FLAGS}")
set(CMAKE_STATIC_LIBRARY_PREFIX "")

# Output vars
set(LIB_TARGET "${PROJECT_NAME}")
set(LIB_STEPPER "${PROJECT_NAME}" PARENT_SCOPE)
set(TEST_LIB "catch")
set(TEST_TARGET "${PROJECT_NAME}-test")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Input vars
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(INC_DIR "${CMAKE_SOURCE_DIR}/inc")
set(TEST_DIR "${CMAKE_SOURCE_DIR}/test")
set(TEST_INC "${TEST_DIR}/inc")

file(GLOB_RECURSE LIB_SOURCES ${SRC_DIR}/*.cpp ${SRC_DIR}/*.c)
add_library(${LIB_TARGET} STATIC ${LIB_SOURCES})
target_include_directories(${LIB_TARGET} PUBLIC ${INC_DIR})
target_link_libraries(${LIB_TARGET} -lpthread)

# The Catch testing library
add_library(${TEST_LIB} INTERFACE)
target_include_directories(${TEST_LIB} INTERFACE ${TEST_INC})

# The test binary
file(GLOB_RECURSE TEST_SOURCES ${TEST_DIR}/*.cpp ${TEST_DIR}/*.c)
add_executable(${TEST_TARGET} ${TEST_SOURCES})
target_link_libraries(${TEST_TARGET} ${TEST_LIB})
target_link_libraries(${TEST_TARGET} ${LIB_TARGET})

# Expose tests to CMake
enable_testing()
add_test(NAME ${TEST_TARGET} COMMAND ${TEST_TARGET})
