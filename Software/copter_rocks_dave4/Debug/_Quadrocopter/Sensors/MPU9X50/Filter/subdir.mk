################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../_Quadrocopter/Sensors/MPU9X50/Filter/QuaternionFilters.c \
../_Quadrocopter/Sensors/MPU9X50/Filter/kalman.c 

OBJS += \
./_Quadrocopter/Sensors/MPU9X50/Filter/QuaternionFilters.o \
./_Quadrocopter/Sensors/MPU9X50/Filter/kalman.o 

C_DEPS += \
./_Quadrocopter/Sensors/MPU9X50/Filter/QuaternionFilters.d \
./_Quadrocopter/Sensors/MPU9X50/Filter/kalman.d 


# Each subdirectory must supply rules for building sources it contributes
_Quadrocopter/Sensors/MPU9X50/Filter/%.o: ../_Quadrocopter/Sensors/MPU9X50/Filter/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM-GCC C Compiler'
	"C:\DAVEv4\DAVE-4.1.2\eclipse\ARM-GCC-49/bin/arm-none-eabi-gcc" -MMD -MT "$@" -DXMC4500_F100x1024 -I"C:\Workspaces\DAVE-4.1\Flightcontroller_PWM_CopterRocks_Working\Libraries\XMCLib\inc" -I"C:\Workspaces\DAVE-4.1\Flightcontroller_PWM_CopterRocks_Working/Libraries/CMSIS/Include" -I"C:\Workspaces\DAVE-4.1\Flightcontroller_PWM_CopterRocks_Working/Libraries/CMSIS/Infineon/XMC4500_series/Include" -I"C:\Workspaces\DAVE-4.1\Flightcontroller_PWM_CopterRocks_Working" -I"C:\Workspaces\DAVE-4.1\Flightcontroller_PWM_CopterRocks_Working\Dave\Generated" -I"C:\Workspaces\DAVE-4.1\Flightcontroller_PWM_CopterRocks_Working\Libraries" -O0 -ffunction-sections -fdata-sections -Wall -std=gnu99 -mfloat-abi=softfp -Wa,-adhlns="$@.lst" -pipe -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $@" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo.

