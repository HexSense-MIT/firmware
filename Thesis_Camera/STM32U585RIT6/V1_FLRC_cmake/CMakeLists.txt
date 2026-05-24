cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME V1_FLRC_cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    Core/Inc/tusb_config.h
    Core/Inc/comm_cmd.h
    Core/Inc/comm.h
    Core/Src/comm.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add TinyUSB (loads tinyusb_target_add function)
add_subdirectory(tinyusb)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Application sources
    Core/Src/usb_descriptors.c
    Core/Src/icm20948.c
    Core/Src/lr2021.c

    # TinyUSB DWC2 driver (STM32U585 uses Synopsys OTG FS)
    tinyusb/portable/synopsys/dwc2/dcd_dwc2.c
    tinyusb/portable/synopsys/dwc2/dwc2_common.c
)

# Add TinyUSB stack sources and includes
tinyusb_target_add(${CMAKE_PROJECT_NAME})

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Enable float/double support in printf (newlib-nano omits it by default)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
