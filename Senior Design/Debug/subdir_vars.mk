################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../EK_TM4C123GXL.cmd 

CFG_SRCS += \
../empty.cfg 

C_SRCS += \
../Accelerometer.c \
../EK_TM4C123GXL.c \
../drive.c \
../empty.c \
../light_sensor.c \
../motors.c 

OBJS += \
./Accelerometer.obj \
./EK_TM4C123GXL.obj \
./drive.obj \
./empty.obj \
./light_sensor.obj \
./motors.obj 

C_DEPS += \
./Accelerometer.pp \
./EK_TM4C123GXL.pp \
./drive.pp \
./empty.pp \
./light_sensor.pp \
./motors.pp 

GEN_MISC_DIRS += \
./configPkg/ 

GEN_CMDS += \
./configPkg/linker.cmd 

GEN_OPTS += \
./configPkg/compiler.opt 

GEN_FILES += \
./configPkg/linker.cmd \
./configPkg/compiler.opt 

GEN_FILES__QUOTED += \
"configPkg\linker.cmd" \
"configPkg\compiler.opt" 

GEN_MISC_DIRS__QUOTED += \
"configPkg\" 

C_DEPS__QUOTED += \
"Accelerometer.pp" \
"EK_TM4C123GXL.pp" \
"drive.pp" \
"empty.pp" \
"light_sensor.pp" \
"motors.pp" 

OBJS__QUOTED += \
"Accelerometer.obj" \
"EK_TM4C123GXL.obj" \
"drive.obj" \
"empty.obj" \
"light_sensor.obj" \
"motors.obj" 

C_SRCS__QUOTED += \
"../Accelerometer.c" \
"../EK_TM4C123GXL.c" \
"../drive.c" \
"../empty.c" \
"../light_sensor.c" \
"../motors.c" 

GEN_CMDS__FLAG += \
-l"./configPkg/linker.cmd" 

GEN_OPTS__FLAG += \
--cmd_file="./configPkg/compiler.opt" 


