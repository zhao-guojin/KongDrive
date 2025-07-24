# THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.25)

# Toolchain setup
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Project setup
project(Odrive C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Global compile options
add_compile_options(
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
        -specs=nano.specs
        -specs=nosys.specs
        -u_printf_float
        -u_scanf_float
        -O0
)

# FPU specific definitions
add_compile_definitions(
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
        USE_STM324xG_EVAL
        USE_USB_OTG_FS
        STM32F40XX
        USE_STDPERIPH_DRIVER
        ARM_MATH_DSP
        ARM_MATH_CM4

)


# Assembler preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Build type configuration
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Release build: Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "RelWithDebInfo build: Optimization with debug info")
    add_compile_options(-Ofast -g3)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "MinSizeRel build: Size optimization")
    add_compile_options(-Os)
else ()
    message(STATUS "Debug build: Minimal optimization with debug info")
    add_compile_options(-Og -g3)
endif ()


# 定义需要递归查找头文件的根目录列表
# 这里列出工程中各个模块的根目录（不局限于 Inc 目录），方便递归查找所有子目录中的头文件
set(HEADER_ROOTS
        "${CMAKE_SOURCE_DIR}/CORE"
        "${CMAKE_SOURCE_DIR}/FWLIB"
        "${CMAKE_SOURCE_DIR}/SYSTEM"
        "${CMAKE_SOURCE_DIR}/USER"
        "${CMAKE_SOURCE_DIR}/HARDWARE"
        "${CMAKE_SOURCE_DIR}/USB"
        "${CMAKE_SOURCE_DIR}/DSP_LIB"

)

# 用于保存所有独一无二的头文件所在目录
# 初始化一个空变量，后续将把查找到的每个头文件所在目录保存到这里
set(ALL_INCLUDE_DIRS "")

# 遍历每个根目录，ROOT_DIR 依次取 HEADER_ROOTS 中的每个路径
foreach(ROOT_DIR IN LISTS HEADER_ROOTS)
    # 递归查找当前根目录下所有的 .h 和 .hpp 文件
    # 这里使用 file(GLOB_RECURSE) 命令递归查找指定目录下所有扩展名为 .h 或 .hpp 的文件
    file(GLOB_RECURSE HEADERS
            "${ROOT_DIR}/*.h"
            "${ROOT_DIR}/*.hpp"
    )
    # 遍历所有查找到的头文件，变量 HEADER_FILE 将依次保存每个头文件的完整路径
    foreach(HEADER_FILE IN LISTS HEADERS)
        # 获取头文件所在目录
        # 使用 get_filename_component 命令从头文件的完整路径中提取出目录部分
        get_filename_component(HEADER_DIR ${HEADER_FILE} DIRECTORY)
        # 检查目录是否已在列表中，未找到则添加
        # 使用 list(FIND ...) 检查 ALL_INCLUDE_DIRS 列表中是否已经包含 HEADER_DIR，
        # 如果没有找到，则 DIR_INDEX 的值为 -1
        list(FIND ALL_INCLUDE_DIRS ${HEADER_DIR} DIR_INDEX)
        if(DIR_INDEX EQUAL -1)
            # 如果目录未被添加过，则将该目录追加到 ALL_INCLUDE_DIRS 列表中
            list(APPEND ALL_INCLUDE_DIRS ${HEADER_DIR})
        endif()
    endforeach()
endforeach()

# 将所有独一无二的目录添加到编译器的包含路径中
# 使用 include_directories 命令把之前收集到的所有目录添加到头文件搜索路径
include_directories(${ALL_INCLUDE_DIRS})



# Project definitions
add_definitions(
        -DDEBUG
        -DUSE_HAL_DRIVER
        -DSTM32F407xx
)

# Source collection and filtering
file(GLOB_RECURSE SOURCES
        "CORE/*.c"
        "FWLIB/*.c"
        "Startup/*.*"
        "SYSTEM/*.c"
        "USER/*.c"
        "HARDWARE/*.c"
        "USB/*.c"
        "DSP_LIB/*.c"
)

# Filter out unwanted heap implementations


# Linker configuration
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld)

add_link_options(
        -Wl,-gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -T ${LINKER_SCRIPT}
)

# Executable target
add_executable(${PROJECT_NAME}.elf ${SOURCES})


#------------------------------------------

# 在 add_executable 之后添加以下内容

# 为每个源文件生成汇编输出 (.s 文件)
set(ASM_OUTPUT_DIR "${CMAKE_BINARY_DIR}/asm_output")
file(MAKE_DIRECTORY ${ASM_OUTPUT_DIR})

# 添加自定义目标生成汇编文件
add_custom_target(disassemble_all
        COMMENT "Generating assembly files for all sources"
        VERBATIM
)

# 为每个源文件添加汇编生成规则
foreach(src ${SOURCES})
    get_filename_component(src_name ${src} NAME_WE)
    get_filename_component(src_ext ${src} EXT)

    # 只处理 C/C++/汇编源文件
    if(src_ext MATCHES "\\.(c|cpp|cc|cxx|asm|s)$")
        set(asm_file "${ASM_OUTPUT_DIR}/${src_name}${src_ext}.s")

        add_custom_command(
                OUTPUT ${asm_file}
                COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS} -S ${src} -o ${asm_file}
                DEPENDS ${src}
                COMMENT "Generating assembly for ${src}"
                VERBATIM
        )

        add_custom_target(disassemble_${src_name}
                DEPENDS ${asm_file}
                COMMENT "Generated ${asm_file}"
        )

        add_dependencies(disassemble_all disassemble_${src_name})
    endif()
endforeach()

# 为最终的可执行文件生成反汇编
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJDUMP} -d -S $<TARGET_FILE:${PROJECT_NAME}.elf> > ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.dis
        COMMENT "Generating disassembly file: ${PROJECT_NAME}.dis"
)

# 在现有的 POST_BUILD 步骤中添加注释说明
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMENT "Additional build artifacts: HEX, BIN and disassembly files generated"
)


#-------------------------------------------

# Post-build steps
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Generating HEX and BIN files\n"
)