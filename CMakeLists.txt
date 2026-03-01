cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(ti_mspm0_libxr_dev LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MSPM0_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mspm0-sdk")
set(LIBXR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libxr")

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/sysconfig/device_linker.lds")
set(STARTUP_FILE "${MSPM0_SDK_DIR}/examples/nortos/LP_MSPM0G3507/driverlib/empty_driverlib_library/gcc/startup_mspm0g350x_gcc.c")
set(DRIVERLIB_ARCHIVE "${MSPM0_SDK_DIR}/source/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a")

add_executable(ti_mspm0_libxr_dev.elf
  src/main.c
  src/syscalls_stub.c
  src/app_main.cpp
  sysconfig/ti_msp_dl_config.c
  ${STARTUP_FILE}
  ${LIBXR_DIR}/src/core/libxr_time.cpp
  ${LIBXR_DIR}/src/core/libxr_type.cpp
  ${LIBXR_DIR}/src/core/libxr_mem.cpp
  ${LIBXR_DIR}/src/structure/lockfree_list.cpp
  ${LIBXR_DIR}/src/system/timer.cpp
  ${LIBXR_DIR}/driver/mspm0/mspm0_timebase.cpp
  ${LIBXR_DIR}/driver/mspm0/mspm0_gpio.cpp
  ${LIBXR_DIR}/driver/mspm0/mspm0_uart.cpp
  ${LIBXR_DIR}/driver/mspm0/mspm0_spi.cpp
  ${LIBXR_DIR}/driver/mspm0/mspm0_atomic_shim.c
  ${LIBXR_DIR}/src/core/libxr_rw.cpp
  ${LIBXR_DIR}/src/structure/list.cpp
  ${LIBXR_DIR}/src/structure/queue.cpp
  ${LIBXR_DIR}/src/structure/double_buffer.cpp
  ${LIBXR_DIR}/system/None/thread.cpp
  ${LIBXR_DIR}/system/None/libxr_system.cpp
  ${LIBXR_DIR}/system/None/semaphore.cpp
  ${LIBXR_DIR}/system/None/mutex.cpp
)

target_include_directories(ti_mspm0_libxr_dev.elf PRIVATE
  src
  sysconfig
  ${MSPM0_SDK_DIR}/source
  ${MSPM0_SDK_DIR}/source/ti/devices/msp
  ${MSPM0_SDK_DIR}/source/third_party/CMSIS/Core/Include
  ${MSPM0_SDK_DIR}/source/ti/driverlib
  ${MSPM0_SDK_DIR}/source/ti/driverlib/m0p
  ${LIBXR_DIR}/src
  ${LIBXR_DIR}/src/core
  ${LIBXR_DIR}/src/system
  ${LIBXR_DIR}/src/utils
  ${LIBXR_DIR}/src/structure
  ${LIBXR_DIR}/src/middleware
  ${LIBXR_DIR}/src/driver
  ${LIBXR_DIR}/driver/mspm0
  ${LIBXR_DIR}/system/None
)

target_compile_definitions(ti_mspm0_libxr_dev.elf PRIVATE
  LIBXR_NOT_SUPPORT_MUTI_THREAD=1
  __MSPM0G3507__
  __USE_SYSCONFIG__
  LIBXR_NO_EIGEN
  LIBXR_DEFAULT_SCALAR=double
  LIBXR_PRINTF_BUFFER_SIZE=128
  LIBXR_LOG_LEVEL=4
  LIBXR_LOG_OUTPUT_LEVEL=4
  XR_LOG_MESSAGE_MAX_LEN=64
)

target_compile_options(ti_mspm0_libxr_dev.elf PRIVATE
  -mcpu=cortex-m0plus
  -mthumb
  -mfloat-abi=soft
  -ffunction-sections
  -fdata-sections
  $<$<CONFIG:Debug>:-Og>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>
)

target_link_options(ti_mspm0_libxr_dev.elf PRIVATE
  -mcpu=cortex-m0plus
  -mthumb
  -mfloat-abi=soft
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/ti_mspm0_libxr_dev.map
  -T${LINKER_SCRIPT}
)

target_link_libraries(ti_mspm0_libxr_dev.elf PRIVATE
  ${DRIVERLIB_ARCHIVE}
)

find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)

add_custom_command(TARGET ti_mspm0_libxr_dev.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex
    $<TARGET_FILE:ti_mspm0_libxr_dev.elf>
    ${CMAKE_CURRENT_BINARY_DIR}/ti_mspm0_libxr_dev.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary
    $<TARGET_FILE:ti_mspm0_libxr_dev.elf>
    ${CMAKE_CURRENT_BINARY_DIR}/ti_mspm0_libxr_dev.bin
)
