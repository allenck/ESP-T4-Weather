# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ram)

set(project_elf ${CMAKE_PROJECT_NAME}.elf)
set(project_bin ${CMAKE_PROJECT_NAME}.bin)
