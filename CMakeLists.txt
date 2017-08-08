cmake_minimum_required(VERSION 3.0)

project(libsbus)

set(SRCS
    SBUS.cpp
    )

set(HEADERS
    SBUS.h
    )

add_library(libsbus ${SRCS} ${HEADERS})
