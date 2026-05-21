# Mobile_Roboter
Codebase for the 'Mobile Roboter' course

# Building and flashing the project 

```bash 
# Prepare the build directory
cmake -B build -G "Ninja" -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug

# Build the project 
cmake --build build

# Flash the project to the STM32
st-flash write build/ArmuroL432.bin 0x08000000
```

# Debugging the project

```bash 
# start st-util
st-util
# Start gdb 
arm-none-eabi-gdb ./build/ArmuroL432.elf
# Set the remote target to use the st-util server
target extended-remote :4242
# Halt and reset the MCU
monitor reset halt
# Flash the selected .elf
load
# Set break points 
break main
# Start debugging 
continue
```

