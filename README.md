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

## Block Diagram

![IMG_0787](https://github.com/vanamvamshikrishna1998/Automated-Plant-Watering-System/assets/161431266/152352fa-b7e0-40c7-b5c6-c4ad6b52c239)


## Code Example

### Node 1: Main File (`main.c`)

```c
void StartSensorTask(void *argument)
{
  /* Infinite loop */
  while(1){
    /* Start the ADC conversion */
    HAL_ADC_Start(&hadc1);
    /* Poll for ADC conversion completion */
    if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
    	osMutexAcquire(moistureMutex1Handle, osWaitForever);
		/* Get the ADC converted value */
		moistureLevel = HAL_ADC_GetValue(&hadc1);
		osMutexRelease(moistureMutex1Handle);
    }
    else{
    	char error_msg[] = "ADC conversion failed!\r\n";
    	HAL_UART_Transmit(&huart1, (uint8_t *)error_msg, strlen(error_msg), HAL_MAX_DELAY);
    }
    /* Stop the ADC conversion */
    HAL_ADC_Stop(&hadc1);
    /* Delay for 1 second */
    HAL_Delay(1000);
  }
}

void StartWateringTask(void *argument)
{
  /* Infinite loop */
  while(1){
    /* Check if moisture level is below threshold */
	osMutexAcquire(moistureMutex1Handle, osWaitForever);
	uint16_t threshold = moistureLevel;
	osMutexRelease(moistureMutex1Handle);
    if(threshold < 30){
      /* Turn on the water pump */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    } else {
      /* Turn off the water pump */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    /* Delay for 1 second */
    HAL_Delay(1000);
  }
}

void StartCanTask(void *argument)
{
  /* Set up CAN message header */
  TxHeader.DLC = 2;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x05; // Unique ID for Node 1

  /* Infinite loop */
  while(1){
    /* Prepare CAN data */
	osMutexAcquire(moistureMutex1Handle, osWaitForever);
    TxData[0] = moistureLevel & 0xFF;
    TxData[1] = (moistureLevel >> 8) & 0xFF;
    osMutexRelease(moistureMutex1Handle);
    /* Transmit CAN message */
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    /* Delay for 1 second */
    HAL_Delay(1000);
  }
}
```

### Main Control Unit: Main File (`main.c`)

```c
void StartCANReceiveTask(void *argument)
{
  uint8_t RxData[2];
  uint32_t RxFifo;
  CAN_RxHeaderTypeDef RxHeader;

  while (1)
  {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, RxFifo) != 0)
    {
      HAL_CAN_GetRxMessage(&hcan, RxFifo, &RxHeader, RxData);

      if (RxHeader.StdId == 0x05)
      {
        osMutexAcquire(moistureLevel_1Handle, osWaitForever);
        moisture_level_1 = RxData[0] | (RxData[1] << 8);
        osMutexRelease(moistureLevel_1Handle);
      }
      if (RxHeader.StdId == 0x0A)
      {
        osMutexAcquire(moistureLevel_2Handle, osWaitForever);
        moisture_level_2 = RxData[0] | (RxData[1] << 8);
        osMutexRelease(moistureLevel_2Handle);
      }
    }
  }
}

void StartLCDUpdateTask(void *argument)
{
  char buffer[16];

  while (1)
  {
    osMutexAcquire(moistureLevel_1Handle, osWaitForever);
    sprintf(buffer, "Moisture 1: %d", moisture_level_1);
    osMutexRelease(moistureLevel_1Handle);
    LCD_Command(0x80);
    LCD_Print(buffer);

    osMutexAcquire(moistureLevel_2Handle, osWaitForever);
    sprintf(buffer, "Moisture 2: %d", moisture_level_2);
    osMutexRelease(moistureLevel_2Handle);
    LCD_Command(0xC0);
    LCD_Print(buffer);

    HAL_Delay(1000);
  }
}
```

## Usage

1. **Clone the Repository**: `git clone https://github.com/vanamvamshikrishna1998/Automated-Plant-Watering-System.git`
2. **Open the Project**: Open the project in your preferred IDE (e.g., STM32CubeIDE).
3. **Build and Flash**: Build the project and flash the firmware to the STM32 microcontrollers.
4. **Connect Hardware**: Connect the hardware components as per the schematic.
5. **Run the System**: Power on the system to start monitoring and watering plants automatically.

## Acknowledgments

- STM32CubeMX and HAL libraries for peripheral configuration.
- FreeRTOS for task management.
- Community forums and online resources for development support.

---

