################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/adi/Developer/STM32CubeIDE/Nx_UDP_Echo_Client/Middlewares/ST/netxduo/common/drivers/ethernet/lan8742/nx_stm32_phy_driver.c 

OBJS += \
./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.o 

C_DEPS += \
./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.o: /Users/adi/Developer/STM32CubeIDE/Nx_UDP_Echo_Client/Middlewares/ST/netxduo/common/drivers/ethernet/lan8742/nx_stm32_phy_driver.c Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DTX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../../Core/Inc -I../../AZURE_RTOS/App -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m4/gnu/inc/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m4/gnu/inc/ -I../../Drivers/BSP/STM32F4xx_Nucleo_144 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Interfaces/Network/Ethernet Phy Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Interfaces-2f-Network-2f-Ethernet-20-Phy-20-Interface-2f-LAN8742_Phy_Interface

clean-Middlewares-2f-Interfaces-2f-Network-2f-Ethernet-20-Phy-20-Interface-2f-LAN8742_Phy_Interface:
	-$(RM) ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.cyclo ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.d ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.o ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.su

.PHONY: clean-Middlewares-2f-Interfaces-2f-Network-2f-Ethernet-20-Phy-20-Interface-2f-LAN8742_Phy_Interface

