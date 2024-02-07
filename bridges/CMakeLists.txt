set(LIBNAME "cf_bridges")

set (BRIDGES_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/can/can.cpp
)

set (BRIDGES_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bridges/can/can.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bridges/can/types.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bridges/serial/serial.hpp
    )


add_library(${LIBNAME} ${BRIDGES_SOURCES} ${BRIDGES_HEADER})
target_include_directories(${LIBNAME} PUBLIC include ${BOOST_INCLUDE_DIR})

# Setting std20 for concepts.
set_property(TARGET ${LIBNAME} PROPERTY CXX_STANDARD 20)
