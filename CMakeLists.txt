# 2026-04-21 - LED candle Zigbee project by Claude
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(led_candle)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE include)
