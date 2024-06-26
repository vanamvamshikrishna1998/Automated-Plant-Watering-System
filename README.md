# Automated Plant Watering System with CAN Bus and FreeRTOS

## Overview

This project implements an automated plant watering system using STM32 microcontrollers, CAN bus communication, and FreeRTOS. The system consists of three main components:
- **Node 1 and Node 2**: Each node monitors soil moisture levels and controls a water pump for individual plants.
- **Main Control Unit**: Receives data from the nodes, processes it, and displays the moisture levels on an LCD.

## Components

### Hardware
- **Microcontrollers**: 
  - 1x STM32F103C8T6 (Main Control Unit)
  - 2x STM32F103C8T6 (Node 1 and Node 2)
- **CAN Transceivers**: 3x MCP2551
- **Soil Moisture Sensors**: 2x (one for each node)
- **Water Pumps**: 2x (one for each node)
- **8x2 LCD Display**: 1x (for the main control unit)
- **Miscellaneous**: Resistors, capacitors, relays, transistors, and wiring components

### Software
- **FreeRTOS**: Real-time operating system for task management
- **STM32CubeMX**: Tool for configuring STM32 peripherals and generating initialization code
- **HAL (Hardware Abstraction Layer)**: For peripheral control

## Project Structure

```
├── Main Control Unit/Core
│   ├── Inc
│   │   ├── FreeRTOSConfig.h
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── ...
│   └── Src
│       ├── freertos.c
│       ├── main.c
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       ├── system_stm32f4xx.c
│       └── ...
├── Drivers
│   ├── CMSIS
│   ├── HAL_Driver
│   └── BSP
├── Middlewares
│   └── Third_Party
│       └── FreeRTOS
│           ├── Source
│           └── include
├── Node1/Core
│   ├── Inc
│   │   ├── FreeRTOSConfig.h
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── ...
│   └── Src
│       ├── freertos.c
│       ├── main.c
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       ├── system_stm32f4xx.c
│       └── ...
├── Drivers
│   ├── CMSIS
│   ├── HAL_Driver
│   └── BSP
├── Middlewares
│   └── Third_Party
│       └── FreeRTOS
│           ├── Source
│           └── include
├── Node2/Core
│   ├── Inc
│   │   ├── FreeRTOSConfig.h
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── ...
│   └── Src
│       ├── freertos.c
│       ├── main.c
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       ├── system_stm32f4xx.c
│       └── ...
├── Drivers
│   ├── CMSIS
│   ├── HAL_Driver
│   └── BSP
├── Middlewares
│   └── Third_Party
│       └── FreeRTOS
│           ├── Source
│           └── include
└── README.md

```

## Setup and Configuration

### Node 1 and Node 2

#### Initialization
- Configure ADC for soil moisture sensor input
- Configure GPIO for water pump control
- Initialize CAN for communication

#### FreeRTOS Tasks
- **SensorTask**: Reads soil moisture level
- **WateringTask**: Controls the water pump based on moisture level
- **CANTask**: Sends moisture data over the CAN bus

### Main Control Unit

#### Initialization
- Initialize CAN for communication
- Initialize GPIO and LCD

#### FreeRTOS Tasks
- **CANReceiveTask**: Receives moisture data from nodes
- **LCDUpdateTask**: Updates the LCD with received moisture levels

## Code Example

### Node 1: Main File (`main.c`)

```c
void StartSensorTask(void *argument) {
    while (1) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            moistureLevel = HAL_ADC_GetValue(&hadc1);
        } else {
            char errorMsg[] = "ADC conversion failed!\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
        }
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(1000);
    }
}

void StartWateringTask(void *argument) {
    while (1) {
        if (moistureLevel < 30) {
            HAL_GPIO_WritePin(GPIOA, WATER_PUMP_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOA, WATER_PUMP_PIN, GPIO_PIN_RESET);
        }
        osDelay(1000);
    }
}

void StartCANTask(void *argument) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[2];

    TxHeader.DLC = 2;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x321;

    while (1) {
        TxData[0] = moistureLevel & 0xFF;
        TxData[1] = (moistureLevel >> 8) & 0xFF;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
        osDelay(1000);
    }
}
```

### Main Control Unit: Main File (`main.c`)

```c
void StartCANReceiveTask(void *argument) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[2];

    while (1) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 0) {
            HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
            osMutexAcquire(myMutex, osWaitForever);
            if (RxHeader.StdId == 0x321) {
                moisture_level_1 = RxData[0] | (RxData[1] << 8);
            } else if (RxHeader.StdId == 0x322) {
                moisture_level_2 = RxData[0] | (RxData[1] << 8);
            }
            osMutexRelease(myMutex);
        }
        osDelay(100);
    }
}

void StartLCDUpdateTask(void *argument) {
    char buffer[16];

    while (1) {
        osMutexAcquire(myMutex, osWaitForever);
        sprintf(buffer, "Moisture 1: %d", moisture_level_1);
        LCD_Command(0x80);
        LCD_Print(buffer);
        sprintf(buffer, "Moisture 2: %d", moisture_level_2);
        LCD_Command(0xC0);
        LCD_Print(buffer);
        osMutexRelease(myMutex);
        osDelay(1000);
    }
}
```

## Usage

1. **Clone the Repository**: `git clone https://github.com/your-username/automated-plant-watering-system.git`
2. **Open the Project**: Open the project in your preferred IDE (e.g., STM32CubeIDE).
3. **Build and Flash**: Build the project and flash the firmware to the STM32 microcontrollers.
4. **Connect Hardware**: Connect the hardware components as per the schematic.
5. **Run the System**: Power on the system to start monitoring and watering plants automatically.

## Acknowledgments

- STM32CubeMX and HAL libraries for peripheral configuration.
- FreeRTOS for task management.
- Community forums and online resources for development support.

---

