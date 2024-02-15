cmake_minimum_required(VERSION 2.6)

project(ble-gate C)

SET(SOURCES main.c)

add_executable(ble-gate ${SOURCES})

SET(CMAKE_INSTALL_PREFIX /)

install(TARGERTS ble-gate
        RUNTIME DESTINATION sbin
)