cmake_minimum_required (VERSION 2.8)
project (Fan)

include_directories(Source/include/plugin_interface)
include_directories(Source/include/common)
include_directories(Source/external_libraries/libsndfile/)
include_directories(./)

set(CMAKE_SHARED_MODULE_PREFIX "")
set(CMAKE_SHARED_MODULE_SUFFIX ".scx")

add_library(Fan MODULE Fan.cpp)

find_library(IOKIT_LIBRARY IOKit)
target_link_libraries(Fan PRIVATE ${IOKIT_LIBRARY})
install(TARGETS Fan DESTINATION .)
