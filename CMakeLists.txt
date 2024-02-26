project(cf_motors)

cmake_minimum_required(VERSION 3.2)


find_package(Boost COMPONENTS program_options serialization filesystem system REQUIRED)

add_subdirectory(bridges)
add_subdirectory(protocol)
add_subdirectory(cmd)
