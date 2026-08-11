cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RoboMaster_Infantry)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# 收集所有源文件
file(GLOB_RECURSE CORE_SRCS "Core/Src/*.c")
file(GLOB_RECURSE CORE_CPP_SRCS "Core/Src/*.cpp")  # 修复：改为.cpp文件
file(GLOB_RECURSE HARDWARE_SRCS "HardWare/*.c")
file(GLOB_RECURSE MOTOR_SRCS "Motor/*.c")
file(GLOB_RECURSE APP_TASK_SRCS "APP/APP_Task/src/*.cpp")
file(GLOB_RECURSE APP_SRCS "APP/src/*.cpp")
file(GLOB_RECURSE APP_C_SRCS "APP/src/*.c")  # 新增：收集APP目录下的C文件
file(GLOB_RECURSE BSP_SRCS "BSP/*.c")

# 排除 Core/Src/system_stm32h7xx.c，避免重复定义
list(FILTER CORE_SRCS EXCLUDE REGEX "system_stm32h7xx\\.c$")

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# FPU flags (must match FreeRTOS + startup + linked libs)
set(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv5-sp-d16")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og ${FPU_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FPU_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${FPU_FLAGS}")



# 添加所有源文件到可执行文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CORE_SRCS}
    ${CORE_CPP_SRCS}  # 添加这一行：包含C++源文件
    ${HARDWARE_SRCS}
    ${MOTOR_SRCS}
    ${APP_TASK_SRCS}
    ${APP_SRCS}
    ${APP_C_SRCS}     # 新增：包含APP目录的C文件
    ${BSP_SRCS}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 添加头文件搜索路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    HardWare  # 确保这个路径存在
    APP/APP_Task/inc
    APP/inc
    BSP
    cmake/stm32cubemx/Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USE_HAL_DRIVER
    STM32H7xx
    CORE_CM7  # 添加CM7核心定义
)

# Add linked libraries
# 确保启用C++标准库支持
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # 如果需要，添加标准库支持
    # -lstdc++
)

# 确保C++标准设置正确
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Add user defined libraries


# 设置编译选项
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    -Wno-unused-function
)

# 添加C++特定的编译选项
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
)

# 确保C和C++之间的正确链接
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINKER_LANGUAGE C  # 修复：改为C语言链接器
)

# 配置CMSIS DSP库
set(CMSIS_DSP_PATH ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP)
set(CMSIS_DSP_INCLUDE ${CMSIS_DSP_PATH}/Include)
set(CMSIS_DSP_LIB ${CMSIS_DSP_PATH}/Lib/GCC)

# 添加CMSIS DSP库的头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMSIS_DSP_INCLUDE}
)

# 链接CMSIS DSP库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMSIS_DSP_LIB}/libarm_cortexM7lfsp_math.a
)