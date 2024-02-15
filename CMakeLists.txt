cmake_minimum_required(VERSION 3.5)

PROJECT(ble-gate C)

SET(SOURCES main.c)

ADD_EXECUTABLE(ble-gate ${SOURCES})

SET(CMAKE_INSTALL_PREFIX /)

INSTALL(TARGERTS ble-gate
        RUNTIME DESTINATION sbin
)