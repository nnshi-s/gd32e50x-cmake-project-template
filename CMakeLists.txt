# Reference: https://dev.to/younup/cmake-on-stm32-the-beginning-3766
cmake_minimum_required(VERSION 3.19)

set(CMAKE_TOOLCHAIN_FILE arm-none-eabi-gcc.cmake)

project(gd32-cmake-template)



enable_language(C CXX ASM)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compiler optimization flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -Os")
set(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -Os")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3 -g")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -O3 -g")

add_subdirectory(Firmware/GD32E50x)

# user source files
set(user_source
        Src/main.c
        Src/gd32e50x_it.c
        Src/gd32e507z_eval.c
        Src/systick.c
)

set(executable ${PROJECT_NAME}.elf)

add_executable(${executable} ${user_source})

target_include_directories(${executable} PRIVATE Src)

target_link_libraries(${executable} PRIVATE ${firmware-library-name})

target_compile_options(${executable} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
target_compile_options(${executable} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>)
target_compile_options(${executable} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>)
target_compile_options(${executable} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wuseless-cast>)
target_compile_options(${executable} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wsuggest-override>)



# print executable size
add_custom_command(TARGET ${executable}
        POST_BUILD
        COMMAND arm-none-eabi-size ${executable}
)

# create .hex & .bin file
add_custom_command(TARGET ${executable}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${executable} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${executable} ${PROJECT_NAME}.bin
)