set(PI_PROJECT_NAME "libstepper-pi")
project(${PI_PROJECT_NAME})
####################################################################################################

set(PI_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(PI_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc")
set(PI_INC_DIR "${PI_INC_DIR}" PARENT_SCOPE)
set(PI_LIB_TARGET "${PI_PROJECT_NAME}")
set(PI_LIB_TARGET "${PI_LIB_TARGET}" PARENT_SCOPE)
set(RASPBERRY_PI_GPIO_LIB "-lwiringPi")
set(RASPBERRY_PI_GPIO_LIB "${RASPBERRY_PI_GPIO_LIB}" PARENT_SCOPE)

########################################## Artifact destinations ##################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/pi/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/pi/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/pi/bin)

########################################## Artifact recipes #######################################
# The pi-specific shared logic lib
file(GLOB_RECURSE PI_LIB_SOURCES ${PI_SRC_DIR}/*.cpp ${PI_SRC_DIR}/*.c)
add_library(${PI_LIB_TARGET} STATIC ${PI_LIB_SOURCES})
target_include_directories(${PI_LIB_TARGET} PUBLIC ${PI_INC_DIR})
target_include_directories(${PI_LIB_TARGET} PUBLIC ${CORE_INC_DIR})
target_link_libraries(${PI_LIB_TARGET} ${CORE_LIB_TARGET})
