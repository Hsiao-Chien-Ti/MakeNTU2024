################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/adi/Developer/STM32CubeIDE/Nx_UDP_Echo_Client/AZURE_RTOS/App/app_azure_rtos.c 

OBJS += \
./Application/User/AZURE_RTOS/App/app_azure_rtos.o 

C_DEPS += \
./Application/User/AZURE_RTOS/App/app_azure_rtos.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/AZURE_RTOS/App/app_azure_rtos.o: /Users/adi/Developer/STM32CubeIDE/Nx_UDP_Echo_Client/AZURE_RTOS/App/app_azure_rtos.c Application/User/AZURE_RTOS/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DTX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../../Core/Inc -I../../AZURE_RTOS/App -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m4/gnu/inc/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m4/gnu/inc/ -I../../Drivers/BSP/STM32F4xx_Nucleo_144 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-AZURE_RTOS-2f-App

clean-Application-2f-User-2f-AZURE_RTOS-2f-App:
	-$(RM) ./Application/User/AZURE_RTOS/App/app_azure_rtos.cyclo ./Application/User/AZURE_RTOS/App/app_azure_rtos.d ./Application/User/AZURE_RTOS/App/app_azure_rtos.o ./Application/User/AZURE_RTOS/App/app_azure_rtos.su

.PHONY: clean-Application-2f-User-2f-AZURE_RTOS-2f-App

