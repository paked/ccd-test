cmake_minimum_required(VERSION 3.0)
project(ccdtest)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++11")
    if(NOT WIN32)
        set(GLAD_LIBRARIES dl)
    endif()
endif()

add_subdirectory(vendor/libccd)

#include_directories(
#    vendor/libccd/src/
#    vendor/libccd/src/ccd)

#file(GLOB VENDOR_HEADERS vendor/libccd/src/ccd/config.h)

file(GLOB PROJECT_SOURCES main.cpp)

file(GLOB PROJECT_CONFIGS
    CMakeLists.txt
    README.md
    .gitattributes
    .gitignore
    .gitmodules)

source_group("headers" FILES ${VENDOR_HEADERS})
source_group("sources" FILES ${PROJECT_SOURCES})
source_group("vendors" FILES ${VENDOR_SOURCES})

add_definitions(-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_CONFIGS}
    ${VENDOR_HEADERS} ${VENDOR_SOURCES})

target_link_libraries(${PROJECT_NAME} ccd)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME})
