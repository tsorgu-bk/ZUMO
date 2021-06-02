set(CWARN                   "-Wall -Wextra -Werror") #-Wstrict-prototypes
set(CXXWARN                 "-Wall -Wextra -Werror -Wno-error=unused-variable")
set(CTUNING                 "-fomit-frame-pointer -ffunction-sections -fdata-sections")

ReadVariables(${CMAKE_CURRENT_LIST_DIR}/Makefile "CPU")             # get flag -DCPU = -mcpu=cortex-mXX
ReadVariables(${CMAKE_CURRENT_LIST_DIR}/Makefile "FPU")
ReadVariables(${CMAKE_CURRENT_LIST_DIR}/Makefile "FLOAT-ABI")
ReadVariables(${CMAKE_CURRENT_LIST_DIR}/Makefile "C_DEFS")          # get flag -DSTM32F***
ReadVariables(${CMAKE_CURRENT_LIST_DIR}/Makefile "LDSCRIPT")      # get LD file

string(REPLACE              ";" " " C_DEFS "${C_DEFS}")

if (NOT "${FPU}" STREQUAL "")
    set(ARMFLOAT            "-mfpu=${FPU} -mfloat-abi=${FLOAT-ABI}")
endif()

set(CMCU                    "-mcpu=${CPU} -mthumb ${ARMFLOAT}")

set(CMAKE_C_FLAGS           "${CMAKE_C_FLAGS} -std=gnu11 ${CWARN} ${CTUNING} ${CMCU} ${C_DEFS}")
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=gnu++17 -fno-exceptions ${CXXWARN} ${CTUNING} ${CMCU} ${C_DEFS} -Wno-missing-field-initializers -fno-permissive")

set(PLATFORM_LINKER_SCRIPT  ${CMAKE_CURRENT_LIST_DIR}/${LDSCRIPT})

set(DEVICE                  STM32F405RG)
set(STLINK_CONFIG_FILE      stm32f4x.cfg)
set(BIN_ADDRESS             0x8000000)