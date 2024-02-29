set(EXEC_NAME "fc_motor")


set (
  CLI_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/options.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/builder.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/commands/base.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/commands/rotate.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/commands/status.hpp
)

set (CLI_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/options.cpp
  main.cpp
)

add_executable(${EXEC_NAME} 
  ${CLI_HEADERS}
  ${CLI_SOURCES}
)

target_include_directories(${EXEC_NAME} PRIVATE 
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXEC_NAME}
  protocol
  cf_bridges
  protocol
  ${Boost_LIBRARIES}
)

set_property(TARGET ${EXEC_NAME} PROPERTY CXX_STANDARD 20)

