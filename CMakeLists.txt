cmake_minimum_required(VERSION 3.22)
project(LooseCoupled)

set(CMAKE_CXX_STANDARD 14)

add_executable(LooseCoupled main.cpp test.cc test.h)
