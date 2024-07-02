/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define RS_PIN GPIO_PIN_2
#define EN_PIN GPIO_PIN_3
#define D4_PIN GPIO_PIN_4
#define D5_PIN GPIO_PIN_5
#define D6_PIN GPIO_PIN_6
#define D7_PIN GPIO_PIN_7

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* Definitions for CANReceiveTask */
osThreadId_t CANReceiveTaskHandle;
uint16_t moisture_level_1, moisture_level_2;

const osThreadAttr_t CANReceiveTask_attributes = {
  .name = "CANReceiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for LCDUpdateTask */
osThreadId_t LCDUpdateTaskHandle;
const osThreadAttr_t LCDUpdateTask_attributes = {
  .name = "LCDUpdateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for moistureLevel_1 mutex */
osMutexId_t moistureLevel_1Handle;
const osMutexAttr_t moistureLevel_1_attributes = {
  .name = "moistureLevel_1"
};

/* Definitions for moistureLevel_2 mutex */
osMutexId_t moistureLevel_2Handle;
const osMutexAttr_t moistureLevel_2_attributes = {
  .name = "moistureLevel_2"
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void LCD_Init(void);
void LCD_Command(uint8_t cmd);
void LCD_Print(char *str);
void StartCANReceiveTask(void *argument);
void StartLCDUpdateTask(void *argument);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Initialize the HAL Library */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  LCD_Init();

  /* Initialize the kernel and create mutexes */
  osKernelInitialize();

  /* Create the mutexes */
  moistureLevel_1Handle = osMutexNew(&moistureLevel_1_attributes);
  moistureLevel_2Handle = osMutexNew(&moistureLevel_2_attributes);

  /* Create the tasks */
  CANReceiveTaskHandle = osThreadNew(StartCANReceiveTask, NULL, &CANReceiveTask_attributes);
  LCDUpdateTaskHandle = osThreadNew(StartLCDUpdateTask, NULL, &LCDUpdateTask_attributes);

  /* Start the scheduler */
  osKernelStart();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure GPIO pins */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                        |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void LCD_Init(void)
{
  HAL_Delay(20); // Wait for LCD to stabilize
  LCD_Command(0x20); // Set LCD to 4-bit mode
  LCD_Command(0x28); // 2 lines, 5x7 matrix
  LCD_Command(0x0C); // Display on, cursor off
  LCD_Command(0x06); // Increment cursor
  LCD_Command(0x01); // Clear display
  HAL_Delay(20);
}

/**
  * @brief Sends a command to the LCD
  * @param cmd: Command to be sent
  * @retval None
  *
  */
void LCD_Command(uint8_t cmd)
{
  HAL_GPIO_WritePin(GPIOA, RS_PIN, GPIO_PIN_RESET); // RS = 0 for command
  HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_SET); // Enable high
  HAL_GPIO_WritePin(GPIOA, D4_PIN, (cmd & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, D5_PIN, (cmd & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, D6_PIN, (cmd & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, D7_PIN, (cmd & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_RESET); // Enable low
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_SET); // Enable high
  HAL_GPIO_WritePin(GPIOA, D4_PIN, (cmd & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, D5_PIN, (cmd & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, D6_PIN, (cmd & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, D7_PIN, (cmd & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_RESET); // Enable low
  HAL_Delay(1);
}

/**
  * @brief Prints a string on the LCD
  * @param str: String to be printed
  * @retval None
  */
void LCD_Print(char *str)
{
  while (*str)
  {
    HAL_GPIO_WritePin(GPIOA, RS_PIN, GPIO_PIN_SET); // RS = 1 for data
    HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_SET); // Enable high
    HAL_GPIO_WritePin(GPIOA, D4_PIN, (*str & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D5_PIN, (*str & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D6_PIN, (*str & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D7_PIN, (*str & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_RESET); // Enable low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_SET); // Enable high
    HAL_GPIO_WritePin(GPIOA, D4_PIN, (*str & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D5_PIN, (*str & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D6_PIN, (*str & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D7_PIN, (*str & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, EN_PIN, GPIO_PIN_RESET); // Enable low
    HAL_Delay(1);
    str++;
  }
}

/**
  * @brief  Function implementing the CANReceiveTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartCANReceiveTask(void *argument)
{
  uint8_t RxData[2];
  uint32_t RxFifo = CAN_RX_FIFO0;
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

/**
  * @brief Function implementing the LCDUpdateTask thread.
  * @param argument: Not used
  * @retval None
  */
void StartLCDUpdateTask(void *argument)
{
  char buffer[16];

  while (1)
  {
    osMutexAcquire(moistureLevel_1Handle, osWaitForever);
    sprintf(buffer, "M_1: %d", moisture_level_1);
    osMutexRelease(moistureLevel_1Handle);
    LCD_Command(0x80);
    LCD_Print(buffer);

    osMutexAcquire(moistureLevel_2Handle, osWaitForever);
    sprintf(buffer, "M_2: %d", moisture_level_2);
    osMutexRelease(moistureLevel_2Handle);
    LCD_Command(0xC0);
    LCD_Print(buffer);

    HAL_Delay(1000);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
