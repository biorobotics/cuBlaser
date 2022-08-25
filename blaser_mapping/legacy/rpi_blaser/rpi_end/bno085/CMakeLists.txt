cmake_minimum_required(VERSION 2.8.9)
project(BNO08X)

set(CMAKE_C_STANDARD 11)

# Include directories
include_directories(
    include
    sh2
)

# Source files
file(GLOB INTLIB_SRC src/interrupt.c)

# Interrupt Library
add_library(
    intlib
    ${INTLIB_SRC}
    # include/interrupt.h
)

# BNO08X Driver Library
add_library(
    bno08x_bcm2835
    sh2/sh2_SensorValue.c
    sh2/sh2_util.c
    sh2/sh2.c
    sh2/shtp.c
    src/sh2_hal_bcm2835.c
)
target_link_libraries(
    bno08x_bcm2835
    bcm2835
    intlib
)

# add_executable(bno08x_demo ${BNO08X_SRC} src/bno08x_demo.c)

# Interrupt Library test case
add_executable(interrupt_test src/tests/interrupt_test.c)
target_link_libraries(interrupt_test PRIVATE intlib pthread bcm2835)

# BNO08X IMU Driver test case
add_executable(bno08x_test src/tests/bno08x_test.c)
target_link_libraries(bno08x_test PRIVATE intlib pthread bcm2835 bno08x_bcm2835)
