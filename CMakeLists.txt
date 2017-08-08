cmake_minimum_required(VERSION 3.0)

project(sbus)

set(SRCS
    SBUS.cpp
    )

set(HEADERS
    SBUS.h
    )

add_library(sbus STATIC ${SRCS} ${HEADERS})
