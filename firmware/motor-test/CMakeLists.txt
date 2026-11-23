cmake_minimum_required(VERSION 3.20.0)
list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/pwm.conf")
list(APPEND EXTRA_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/pwm.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_test)

target_sources(app PRIVATE src/main.c)

option(MOTOR_STEP_TEST "Enable the encoder step response test" OFF)
if(MOTOR_STEP_TEST)
  target_compile_definitions(app PRIVATE MOTOR_STEP_TEST)
endif()
