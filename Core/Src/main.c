/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdint.h>
#include<stdbool.h>
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <ee.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
enum {
  BAUD_9600 =  0,
  BAUD_14400 = 1,
  BAUD_19200 = 2
};

#define DEFAULT_ID        (1)
#define DEFAULT_BAUD_RATE BAUD_9600

struct {
  uint16_t id;
  uint16_t baud_rate;
} holding_register;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState active;
} Coil_Info_t;

const Coil_Info_t coil_address_to_gpio[] = {
    { F1_GPIO_Port, F1_Pin, GPIO_PIN_SET },
    { F2_GPIO_Port, F2_Pin, GPIO_PIN_RESET },
    { F3_GPIO_Port, F3_Pin, GPIO_PIN_SET },
    { F4_GPIO_Port, F4_Pin, GPIO_PIN_RESET }
};
const uint8_t coil_length = sizeof(coil_address_to_gpio) / sizeof(Coil_Info_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t CRC16(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];        // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                        // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }

  return crc;
}

uint8_t modbus_state = 0;
uint32_t last_req = 0;
uint8_t function_code = 0;
uint16_t start_address = 0;
uint16_t quantity_or_value = 0;
uint16_t crc_req = 0;
uint8_t byte_count = 0;
uint8_t *data_value = NULL;
uint8_t data_value_index_and_length = 0;
bool req_process_modbus_flag = false;
uint16_t crc_check = 0;

const enum {
  READ_COIL = 1,
  WRITE_SINGLE_COIL = 5,
  READ_HOLDING_REGISTER = 3,
  WRITE_HOLDING_REGISTER = 6,
  WRITE_MULTIPLE_COILS = 15
} Function_Code_t;

uint8_t modbus_data_buffer[20];
uint8_t modbus_data_buffer_index = 0;

void ModbusSlaveProcess(UART_HandleTypeDef *huart) {
  if ((HAL_GetTick() - last_req) > 100) { // if last data more 100mS so reset to state 0
    modbus_state = 0;
    modbus_data_buffer_index = 0;
  }
  last_req = HAL_GetTick();

  uint8_t c = huart->Instance->RDR;
  modbus_data_buffer[modbus_data_buffer_index++] = c;
  if (modbus_data_buffer_index >= sizeof(modbus_data_buffer)) { // Check buffer overflow
    modbus_data_buffer_index = 0;
    modbus_state = 0;
  }

  if (modbus_state == 0) { // Devices Address
    if (c == holding_register.id) {
      function_code = 0;
      start_address = 0;
      quantity_or_value = 0;
      crc_req = 0;
      modbus_state = 1;
    }
  } else if (modbus_state == 1) { // Function code
    function_code = c;
    switch (function_code) {
      case READ_COIL: // Read Coil Status
      case WRITE_SINGLE_COIL: // Write Single Coil
      case READ_HOLDING_REGISTER: // Read Holding Register
      case WRITE_HOLDING_REGISTER: // Write Holding Register
      case WRITE_MULTIPLE_COILS: // Write Multiple Coils
        modbus_state = 2;
        break;

      default:
        modbus_state = 0;
    }
  } else if (modbus_state == 2) { // Start Address HIGH
    start_address |= ((uint16_t) c) << 8;
    modbus_state = 3;
  } else if (modbus_state == 3) { // Start Address LOW
    start_address |= c;
    modbus_state = 4;
  } else if (modbus_state == 4) { // Quantity HIGH / Value HIGH
    quantity_or_value |= ((uint16_t) c) << 8;
    modbus_state = 5;
  } else if (modbus_state == 5) { // Quantity LOW / Value HIGH
    quantity_or_value |= c;
    if (function_code == WRITE_MULTIPLE_COILS) {
      modbus_state = 10;
    } else {
      modbus_state = 6;
    }
  } else if (modbus_state == 6) { // CRC LOW
    crc_req |= c;
    modbus_state = 7;
  } else if (modbus_state == 7) { // CRC HIGH
    crc_req |= ((uint16_t) c) << 8;
    crc_check = CRC16(modbus_data_buffer, modbus_data_buffer_index - 2);
    if (crc_check == crc_req) { // Check CRC
      req_process_modbus_flag = true;
    }
    modbus_state = 0;
  } else if (modbus_state == 10) {
    byte_count = c;
    if (data_value) {
      free(data_value);
      data_value = NULL;
    }
    if (byte_count > 0) {
      data_value = (uint8_t*) malloc(byte_count);
      data_value_index_and_length = 0;
      modbus_state = 11;
    } else {
      modbus_state = 6;
    }
  } else if (modbus_state == 11) {
    data_value[data_value_index_and_length++] = c;
    if (data_value_index_and_length == byte_count) {
      modbus_state = 6;
    }
  }
}

void RS485_Transmit(uint8_t *data, uint16_t len) {
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET); // DE/RE => HIGH
  HAL_UART_Transmit(&huart2, data, len, 1000);
  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET); // DE/RE => LOW
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  ee_init();
  // Read settings from EEPROM
  memset(&holding_register, 0, sizeof(holding_register));

  ee_read(0, sizeof(holding_register), (uint8_t*) &holding_register);

  uint16_t crc_read = 0;
  ee_read(0 + sizeof(holding_register), 2, (uint8_t*) &crc_read);
  uint16_t crc_calc = CRC16((uint8_t*) &holding_register, sizeof(holding_register));
  if (crc_calc != crc_read) {
    ee_format(false); // format all if data is invaild

    // Use default
    holding_register.id = DEFAULT_ID;
    holding_register.baud_rate = DEFAULT_BAUD_RATE;
  }

  if (holding_register.id > 127) {
    holding_register.id = 0;
  }
  if (holding_register.baud_rate > BAUD_19200) {
    holding_register.baud_rate = BAUD_9600;
  }

  // UART setup
  static uint8_t dummy;
  HAL_UART_Receive_IT(&huart2, &dummy, 1);
  huart2.RxISR = ModbusSlaveProcess;

  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET); // DE/RE => LOW
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	  if (req_process_modbus_flag && (function_code == WRITE_SINGLE_COIL)) {
	    uint16_t address = start_address;
	    if ((address >= 0x0000) && (address <= (coil_length - 1))) {
	      bool isON = quantity_or_value == 0xFF00;
	      Coil_Info_t coil = coil_address_to_gpio[address];
	      HAL_GPIO_WritePin(coil.port, coil.pin, isON ? coil.active : !coil.active);
	    }

	    { // Response
	      RS485_Transmit(modbus_data_buffer, modbus_data_buffer_index);
	    }

	    req_process_modbus_flag = false;
	  }

	  if (req_process_modbus_flag && (function_code == READ_COIL)) {
	    { // Response
	      uint8_t byte_count = ceil(quantity_or_value / 8.0f);

        uint8_t buff_len = 3 + byte_count + 2; // Device ID, Function Code, Byte Count, [DATA], CRC Low, CRC High
        uint8_t data_buff[buff_len];
        memset(data_buff, 0, buff_len);

        data_buff[0] = holding_register.id; // Device ID
        data_buff[1] = READ_COIL; // Function Code
        data_buff[2] = byte_count; // Byte Count

        uint8_t bit_index = 0;
        uint8_t byte_index = 0;
        for (uint8_t i=0;i<quantity_or_value;i++) {
          uint16_t address = start_address + i;
          if ((address >= 0x0000) && (address <= (coil_length - 1))) {
            Coil_Info_t coil = coil_address_to_gpio[address];
            data_buff[3 + byte_index] |= HAL_GPIO_ReadPin(coil.port, coil.pin) == coil.active ? (1 << bit_index) : 0;
          }
          bit_index++;
          if (bit_index == 8) {
            bit_index = 0;
            byte_index++;
          }
        }

        uint16_t crc = CRC16(data_buff, buff_len - 2);
        data_buff[buff_len - 2] = crc & 0xFF; // CRC Low
        data_buff[buff_len - 1] = (crc >> 8) & 0xFF; // CRC High

        RS485_Transmit(data_buff, buff_len);
	    }

	    req_process_modbus_flag = false;
	  }

	  if (req_process_modbus_flag && (function_code == READ_HOLDING_REGISTER)) {
	    { // Response
        uint8_t res_len = 3 + (quantity_or_value * 2) + 2; // (Device ID, Function Code, Length) + DATA + (CRC Low, CRC High)
        uint8_t data_res_buffer[res_len];
        memset(data_res_buffer, 0, res_len);

        data_res_buffer[0] = holding_register.id; // Device ID
        data_res_buffer[1] = READ_HOLDING_REGISTER; // Function Code
        data_res_buffer[2] = quantity_or_value * 2; // Length

        for (uint8_t i=0;i<quantity_or_value;i++) {
          uint16_t addr = start_address + i;
          int16_t data_i16 = 0;
          if ((addr >= 0x0101) && (addr <= 0x0104)) {
            data_i16 = ((uint16_t*) &holding_register)[addr - 0x0101];
          }
          data_res_buffer[3 + (i * 2) + 0] = (data_i16 >> 8) & 0xFF;
          data_res_buffer[3 + (i * 2) + 1] = data_i16 & 0xFF;
        }

        uint16_t crc = CRC16(data_res_buffer, 3 + (quantity_or_value * 2));
        data_res_buffer[3 + (quantity_or_value * 2) + 0] = crc & 0xFF; // CRC Low
        data_res_buffer[3 + (quantity_or_value * 2) + 1] = (crc >> 8) & 0xFF; // CRC High

        RS485_Transmit(data_res_buffer, res_len);
	    }

	    req_process_modbus_flag = false;
	  }

	  if (req_process_modbus_flag && (function_code == WRITE_HOLDING_REGISTER)) {
	    if ((start_address >= 0x0101) && (start_address <= 0x0104)) {
	      ((uint16_t*) &holding_register)[start_address - 0x0101] = quantity_or_value;

	      // Save to EEPROM
	      uint8_t buff_8byte[8 * 2];
	      memset(buff_8byte, 0, sizeof(buff_8byte));
	      memcpy(&buff_8byte[0], &holding_register, sizeof(holding_register));
	      uint16_t crc = CRC16((uint8_t*) &holding_register, sizeof(holding_register));
	      memcpy(&buff_8byte[0 + sizeof(holding_register)], &crc, 2);

	      ee_format(false); // format all before write new data
	      ee_write(0, sizeof(buff_8byte), buff_8byte);
	    }

	    { // Response
	      int16_t data_i16 = 0;
	      if ((start_address >= 0x0101) && (start_address <= 0x0104)) {
	        data_i16 = ((uint16_t*) &holding_register)[start_address - 0x0101];
	      }

	      uint8_t data_buff[] = {
	          holding_register.id, // Device ID
	          WRITE_HOLDING_REGISTER, // Function Code
	          2, // Length
	          (uint8_t)(data_i16 >> 8), // Start Address Hi
	          (uint8_t)(data_i16 & 0xFF), // Start Address Lo
	          0x00, // CRC Lo
	          0x00 // CRC Hi
	      };

	      uint16_t crc = CRC16(data_buff, 6);
	      data_buff[sizeof(data_buff) - 2] = crc & 0xFF; // CRC Low
	      data_buff[sizeof(data_buff) - 1] = (crc >> 8) & 0xFF; // CRC High

	      RS485_Transmit(data_buff, sizeof(data_buff));
	    }

	    req_process_modbus_flag = false;
	  }

	  if (req_process_modbus_flag && (function_code == WRITE_MULTIPLE_COILS)) {
	    uint8_t bit_index = 0;
	    uint8_t byte_index = 0;
	    for (uint8_t i=0;i<quantity_or_value;i++) {
	      uint16_t address = start_address + i;
	      if ((address >= 0x0000) && (address <= (coil_length - 1))) {
	        bool isON = data_value[byte_index] & (1 << bit_index);
	        Coil_Info_t coil = coil_address_to_gpio[address];
	        HAL_GPIO_WritePin(coil.port, coil.pin, isON ? coil.active : !coil.active);
	      }
	      bit_index++;
	      if (bit_index == 8) {
	        bit_index = 0;
	        byte_index++;
	        if (byte_index == data_value_index_and_length) {
	          break;
	        }
	      }
	    }

	    { // Response
        uint8_t data_buff[] = {
            holding_register.id, // Device ID
            WRITE_MULTIPLE_COILS, // Function Code
            (uint8_t)(start_address >> 8), // Start Address Hi
            (uint8_t)(start_address & 0xFF), // Start Address Lo
            (uint8_t)(quantity_or_value >> 8), // Quantity of Outputs Hi
            (uint8_t)(quantity_or_value & 0xFF), // Quantity of Outputs Lo
            0x00, // CRC Lo
            0x00 // CRC Hi
        };

        uint16_t crc = CRC16(data_buff, 6);
        data_buff[sizeof(data_buff) - 2] = crc & 0xFF; // CRC Low
        data_buff[sizeof(data_buff) - 1] = (crc >> 8) & 0xFF; // CRC High

        RS485_Transmit(data_buff, sizeof(data_buff));
	    }

	    req_process_modbus_flag = false;
	  }

	  HAL_IWDG_Refresh(&hiwdg);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 599;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_DIR_Pin|F1_Pin|F3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, F2_Pin|F4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RS485_DIR_Pin F1_Pin F2_Pin F3_Pin
                           F4_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_Pin|F1_Pin|F2_Pin|F3_Pin
                          |F4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
