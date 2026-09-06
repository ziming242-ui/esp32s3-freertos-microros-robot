cmake_minimum_required(VERSION 3.16)

# PlatformIO treats src/ as the application component automatically.  Native
# ESP-IDF builds (used by the official micro-ROS Linux toolchain) need the
# directory registered explicitly.
set(EXTRA_COMPONENT_DIRS "src")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only the application component and its recursive dependencies.  This
# avoids compiling unrelated ESP-IDF components during each WSL verification.
idf_build_set_property(MINIMAL_BUILD ON)

project(esp32s3_freertos_basics)
