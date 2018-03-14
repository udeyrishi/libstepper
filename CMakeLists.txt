cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "libstepper")
project(${PROJECT_NAME})
####################################################################################################
####################################################################################################

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

#########################################Build flags################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_C_STANDARD_REQUIRED on)
set(DEBUG_BUILD_FLAGS "-Wall -Wextra -Wsign-conversion -pedantic")
set(RELEASE_BUILD_FLAGS "${DEBUG_BUILD_FLAGS} -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${RELEASE_BUILD_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${DEBUG_BUILD_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${RELEASE_BUILD_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${DEBUG_BUILD_FLAGS}")

########################################## Artifact destinations ##################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_STATIC_LIBRARY_PREFIX "")

#################################### The testing libary of choice #################################
set(TEST_LIB "catch")
set(TEST_INC "${CMAKE_CURRENT_SOURCE_DIR}/lib/${TEST_LIB}/inc")
set(TEST_RUNNER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/lib/${TEST_LIB}/src/runner.cpp")
set(TEST_RUNNER_LIB "${TEST_LIB}-runner")

##################################### Artifact inputs/outputs #####################################
# The Catch testing library
add_library(${TEST_LIB} INTERFACE)
target_include_directories(${TEST_LIB} INTERFACE ${TEST_INC})

# The shared test runner lib. Mostly, because runner.cpp takes a while to compile to the pi.
add_library(${TEST_RUNNER_LIB} STATIC ${TEST_RUNNER_SRC})
target_include_directories(${TEST_RUNNER_LIB} PUBLIC ${TEST_INC})

add_subdirectory(core)
add_subdirectory(pi)

# Actual executable
set(MAIN "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
set(BIN_TARGET ${PROJECT_NAME}-demo)

# The main executable
add_executable(${BIN_TARGET} ${MAIN})
target_link_libraries(${BIN_TARGET} ${CORE_LIB_TARGET})
target_link_libraries(${BIN_TARGET} ${PI_LIB_TARGET})
target_link_libraries(${BIN_TARGET} ${RASPBERRY_PI_GPIO_LIB})

# Expose tests to CMake
enable_testing()
