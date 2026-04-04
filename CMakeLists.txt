cmake_minimum_required(VERSION 3.22)

enable_language(C CXX ASM)

project(GDUT_RC_Library)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_subdirectory(Third_Party/CMSIS-DSP)

target_compile_options(CMSISDSP PRIVATE
  $<$<CONFIG:Debug>:-O0>
  $<$<CONFIG:Debug>:-g3>
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:Release>:-ffast-math>
)
target_compile_definitions(CMSISDSP PRIVATE
  ARM_MATH_CM4
  __FPU_PRESENT=1
)
target_link_libraries(CMSISDSP PRIVATE
  ${TOOLCHAIN_LINK_LIBRARIES}
  stm32cubemx
)

add_library(GDUT_RC_Library
  ${CMAKE_CURRENT_SOURCE_DIR}/BSP/bsp_can.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/BSP/bsp_ps2.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/TLSF/tlsf.c
)
target_include_directories(GDUT_RC_Library PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/BSP
  ${CMAKE_CURRENT_SOURCE_DIR}/Async
  ${CMAKE_CURRENT_SOURCE_DIR}/Components
  ${CMAKE_CURRENT_SOURCE_DIR}/DSP
  ${CMAKE_CURRENT_SOURCE_DIR}/Modules
  ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/TLSF
)
target_link_libraries(GDUT_RC_Library PUBLIC
  STM32_Drivers
  ${TOOLCHAIN_LINK_LIBRARIES}
  FreeRTOS
  CMSISDSP
)
target_compile_definitions(GDUT_RC_Library PUBLIC
)

# add_executable(test test/test.cpp)
# target_link_libraries(test PRIVATE
#   GDUT_RC_Library
# )
