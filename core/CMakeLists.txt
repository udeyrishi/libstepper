set(CORE_PROJECT_NAME "libstepper-core")
project(${CORE_PROJECT_NAME})
####################################################################################################

set(CORE_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(CORE_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc")
set(CORE_INC_DIR "${CORE_INC_DIR}" PARENT_SCOPE)
set(CORE_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test")
set(CORE_LIB_TARGET "${CORE_PROJECT_NAME}")
set(CORE_LIB_TARGET "${CORE_LIB_TARGET}" PARENT_SCOPE)
set(CORE_TEST_TARGET "${CORE_PROJECT_NAME}-test")

########################################## Artifact destinations ##################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/core/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/core/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/core/bin)

########################################## Artifact recipes #######################################
file(GLOB_RECURSE CORE_LIB_SOURCES ${CORE_SRC_DIR}/*.cpp ${CORE_SRC_DIR}/*.c)
add_library(${CORE_LIB_TARGET} STATIC ${CORE_LIB_SOURCES})
target_include_directories(${CORE_LIB_TARGET} PUBLIC ${CORE_INC_DIR})
target_link_libraries(${CORE_LIB_TARGET} -lpthread)

file(GLOB_RECURSE CORE_TEST_SOURCES ${CORE_TEST_DIR}/*.cpp ${CORE_TEST_DIR}/*.c)
add_executable(${CORE_TEST_TARGET} ${CORE_TEST_SOURCES})
target_link_libraries(${CORE_TEST_TARGET} ${TEST_LIB})
target_link_libraries(${CORE_TEST_TARGET} ${TEST_RUNNER_LIB})
target_link_libraries(${CORE_TEST_TARGET} ${CORE_LIB_TARGET})

# Expose tests to CMake
add_test(NAME ${CORE_TEST_TARGET} COMMAND ${CORE_TEST_TARGET})
