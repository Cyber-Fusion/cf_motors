set (LIBNAME "protocol")

set ( SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/can/rmdx.cpp
)

set ( HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/include/protocol/can/rmdx.hpp
)

add_library(${LIBNAME} ${SOURCES} ${HEADERS})

target_include_directories(${LIBNAME} PUBLIC include)
target_link_libraries(${LIBNAME} cf_bridges)
set_property(TARGET ${LIBNAME} PROPERTY CXX_STANDARD 20)
