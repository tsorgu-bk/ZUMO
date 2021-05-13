# CMake template project for STM32, with GoogleTests and STM32CubeMX

# How to use
## Requirements in PATH
- [CMake](https://cmake.org/)
- [program make](https://www.gnu.org/software/make/) or [Ninja](https://ninja-build.org/)
- [gcc, g++](https://gcc.gnu.org/) (in Windows with [MinGW](https://www.youtube.com/watch?v=sXW2VLrQ3Bs))
- [gcc-arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- [git](https://git-scm.com/downloads) - for downloading 

# Run
## STM32CubeMX
- create standard project in STM32CubeMX
- in **Project Manager** Project:
  - Application Structure: Basic
  - Toolchain / IDE: Makefile
- in **Project Manager** Code Generator:
  - Copy only the necessary library files
- copy generated files to target/arm directory

## 1. Command line
- create cmake_build_debug directory and go
- run **cmake -DTARGET=arm -G "MinGW Makefiles" ..**
  - -DTARGET=**arm**(for STM project)/**x86**(for GoogleTests)
  - -DCMAKE_BUILD_TYPE=**debug**/**release**
  - **-G "MinGW Makefiles"** for windows

## 2. CLion
- download and install  [CLion](https://www.jetbrains.com/clion/download/#section=windows)
- File -> Open
- find first **CMakeLists.txt** file in project directory
- **Open as Project**
- File -> Settings -> Build, Execution, Deployment -> CMake -> Build type
  - Release (recomended)
  - Debug
- in Windows add **-G "MinGW Makefiles"**

## 3. VS Code


# SWO debug 
In C or C++ code write:
```C
int _write(int file, char *ptr, int len) {
    (void) file;
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++){
        ITM_SendChar(*ptr++);
    }
    return len;
}
```

And use printf:
```C
printf("Hello world\n");
```

Using openOCD:
```
openocd -f board/stm32f4discovery.cfg -f interface/stlink.cfg -c "transport select hla_swd"  -c "tpiu config internal - uart off clock_rate" -c "itm ports on" -c "tcl_port 6666"
```

Where clock_rate should be changed with number, ex. 180000000 for 180 MHz