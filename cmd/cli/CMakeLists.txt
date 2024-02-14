set(EXEC_NAME "fc_motor")


set (
  CLI_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/options.hpp
)

set (CLI_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/options.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/command.cpp
  main.cpp
)

add_executable(${EXEC_NAME} 
  ${CLI_HEADERS}
  ${CLI_SOURCES}
)

target_include_directories(${EXEC_NAME} PRIVATE 
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXEC_NAME}
  protocol
  cf_bridges
  protocol
  ${Boost_LIBRARIES}
)

set_property(TARGET ${EXEC_NAME} PROPERTY CXX_STANDARD 20)

