cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_library(cQueue STATIC
  cQueue.c
)

target_include_directories(cQueue PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/
)
