cmake_minimum_required(VERSION 3.20.0)

option(MOTOR_TEST_ON_BOOT "Run the conveyor PWM test at startup" OFF)
if(MOTOR_TEST_ON_BOOT)
  list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../motor-test/pwm.conf")
  list(APPEND EXTRA_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../motor-test/pwm.overlay")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(camera_snapshot)

target_sources(app PRIVATE src/main.c src/ov3660.c)

if(MOTOR_TEST_ON_BOOT)
  target_sources(app PRIVATE ../motor-test/src/main.c)
  target_compile_definitions(app PRIVATE MOTOR_TEST_ON_BOOT)
endif()
