project(rib_support LANGUAGES CXX)
cmake_minimum_required(VERSION 3.0.0)

add_library(rib_support SHARED IMPORTED GLOBAL)

set_target_properties(rib_support PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/librib_support.so)
