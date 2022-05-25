cmake_minimum_required(VERSION 3.22)
project(LooseCoupled)

set(CMAKE_CXX_STANDARD 14)

add_executable(LooseCoupled
               src/main.cc
               src/basetk/base_matrix.cc src/basetk/base_matrix.h
               src/basetk/base_time.cc src/basetk/base_time.h
               src/basetk/base_sdc.cc src/basetk/base_sdc.h
               src/basetk/base_math.cc src/basetk/base_math.h
               src/basetk/base_earth.cc src/basetk/base_earth.h
               src/basetk/base_app.cc src/basetk/base_app.h
               src/gnsstk/gnss_app.cc src/gnsstk/gnss_app.h
               src/gnsstk/gnss_file_stream.cc src/gnsstk/gnss_file_stream.h
               src/gnsstk/gnss_spp.cc src/gnsstk/gnss_spp.h
               src/gnsstk/gnss_rtk.cc src/gnsstk/gnss_rtk.h
               src/sinstk/sins_app.cc src/sinstk/sins_app.h
               src/sinstk/sins_file_stream.cc src/sinstk/sins_file_stream.h
               src/sinstk/sins_mechanization.cc src/sinstk/sins_mechanization.h
               src/sinstk/sins_loose_coupled.cc src/sinstk/sins_loose_coupled.h)
