cmake_minimum_required(VERSION 3.10)

project(mc_h1)

set(CMAKE_CXX_STANDARD 17)

find_package(mc_rtc REQUIRED)
find_package(h1_description REQUIRED)

if(${h1_description_FOUND})
  if("${h1_description_INSTALL_PREFIX}" STREQUAL "")
    if("${h1_description_SOURCE_PREFIX}" STREQUAL "")
      message(
        FATAL_ERROR
          "Your h1_description package does not define where to find the data")
    endif()
    set(H1_DESCRIPTION_PATH "${h1_description_SOURCE_PREFIX}")
  else()
    set(H1_DESCRIPTION_PATH "${h1_description_INSTALL_PREFIX}")
  endif()
endif()

message("-- H1_DESCRIPTION_PATH: ${H1_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

macro(add_module NAME)
  add_robot(${NAME} src/${NAME}.cpp src/${NAME}.h)
  target_include_directories(${NAME}
                             PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
endmacro()

add_module(h1)

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
