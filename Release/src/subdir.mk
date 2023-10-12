################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/robofinist2023.cpp 

OBJS += \
./src/robofinist2023.o 

CPP_DEPS += \
./src/robofinist2023.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-ev3liga-linux-gnueabi-g++ -mcpu=arm9 -marm -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -IC:/Users/Pavel/eclipse-workspace/robofinist2023/API/include -std=c++17 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


