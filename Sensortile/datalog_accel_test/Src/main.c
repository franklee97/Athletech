/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "LSM6DSM_ACC_GYRO_driver_HL.h"
#include "LSM6DSM_ACC_GYRO_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SPI_HandleTypeDef SPI_Sensor_Handle;
static DrvContextTypeDef ACCELERO_SensorHandle[2];
static ACCELERO_Data_t ACCELERO_Data[2]; // Accelerometer - all.
static void *LSM6DSM_X_0_handle = NULL;
static LSM6DSM_X_Data_t LSM6DSM_X_0_Data; // Accelerometer - sensor 0.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_SPI2_Init(void);

// keep in main
//DrvStatusTypeDef BSP_ACCELERO_Init(ACCELERO_ID_t id, void **handle);
//static DrvStatusTypeDef BSP_LSM6DSM_ACCELERO_Init(void **handle);
//DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable(void *handle);
//void Accelero_Sensor_Handler(void *handle);
//DrvStatusTypeDef BSP_ACCELERO_Get_Instance(void *handle, uint8_t *instance);
//DrvStatusTypeDef BSP_ACCELERO_IsInitialized(void *handle, uint8_t *status);
//DrvStatusTypeDef BSP_ACCELERO_Get_Axes(void *handle, SensorAxes_t *acceleration);
//
//uint8_t Sensor_IO_SPI_CS_Init(void *handle);
//DrvStatusTypeDef Sensor_IO_SPI_Init(void);
//uint8_t Sensor_IO_SPI_CS_Enable(void *handle);
//uint8_t Sensor_IO_SPI_CS_Disable(void *handle);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
//	MX_SPI2_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	// Initialize sensors
	if (BSP_ACCELERO_Init(LSM6DSM_X_0, &LSM6DSM_X_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}
	// Enable sensor
	BSP_ACCELERO_Sensor_Enable(LSM6DSM_X_0_handle);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		// get data
		Accelero_Sensor_Handler(LSM6DSM_X_0_handle);
//		CDC_Transmit_FS((uint8_t*) "Hello World\r\n", 13);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
//static void MX_SPI2_Init(void) {
//
//	/* USER CODE BEGIN SPI2_Init 0 */
//
//	/* USER CODE END SPI2_Init 0 */
//
//	/* USER CODE BEGIN SPI2_Init 1 */
//
//	/* USER CODE END SPI2_Init 1 */
//	/* SPI2 parameter configuration*/
//	SPI_Sensor_Handle.Instance = SPI2;
//	SPI_Sensor_Handle.Init.Mode = SPI_MODE_MASTER;
//	SPI_Sensor_Handle.Init.Direction = SPI_DIRECTION_1LINE;
//	SPI_Sensor_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
//	SPI_Sensor_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
//	SPI_Sensor_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
//	SPI_Sensor_Handle.Init.NSS = SPI_NSS_SOFT;
//	SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//	SPI_Sensor_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
//	SPI_Sensor_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
//	SPI_Sensor_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//	SPI_Sensor_Handle.Init.CRCPolynomial = 7;
//	SPI_Sensor_Handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//	SPI_Sensor_Handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
//	if (HAL_SPI_Init(&SPI_Sensor_Handle) != HAL_OK) {
//		Error_Handler();
//	}
//	/* USER CODE BEGIN SPI2_Init 2 */
//
//	/* USER CODE END SPI2_Init 2 */
//
//}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Initialize an accelerometer sensor
 * @param id the accelerometer sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Init(ACCELERO_ID_t id, void **handle) {

	*handle = NULL;

	switch ((int) id) {
	case LSM6DSM_X_0: {
		if (BSP_LSM6DSM_ACCELERO_Init(handle) == COMPONENT_ERROR) {
			return COMPONENT_ERROR;
		}
		break;
	}
	}

	return COMPONENT_OK;
}

DrvStatusTypeDef BSP_LSM6DSM_ACCELERO_Init(void **handle) {
	ACCELERO_Drv_t *driver = NULL;
	uint8_t data = 0x0C;

	if (ACCELERO_SensorHandle[LSM6DSM_X_0].isInitialized == 1) {
		/* We have reached the max num of instance for this component */
		return COMPONENT_ERROR;
	}

	if (Sensor_IO_SPI_Init() == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	/* Setup sensor handle. */
	ACCELERO_SensorHandle[LSM6DSM_X_0].who_am_i = LSM6DSM_ACC_GYRO_WHO_AM_I;
	ACCELERO_SensorHandle[LSM6DSM_X_0].ifType = 1; // SPI interface
	ACCELERO_SensorHandle[LSM6DSM_X_0].address =
	LSM6DSM_ACC_GYRO_I2C_ADDRESS_HIGH;
	ACCELERO_SensorHandle[LSM6DSM_X_0].spiDevice = LSM6DSM;
	ACCELERO_SensorHandle[LSM6DSM_X_0].instance = LSM6DSM_X_0;
	ACCELERO_SensorHandle[LSM6DSM_X_0].isInitialized = 0;
	ACCELERO_SensorHandle[LSM6DSM_X_0].isEnabled = 0;
	ACCELERO_SensorHandle[LSM6DSM_X_0].isCombo = 1;
	ACCELERO_SensorHandle[LSM6DSM_X_0].pData =
			(void *) &ACCELERO_Data[LSM6DSM_X_0];
	ACCELERO_SensorHandle[LSM6DSM_X_0].pVTable = (void *) &LSM6DSM_X_Drv;
	ACCELERO_SensorHandle[LSM6DSM_X_0].pExtVTable = (void *) &LSM6DSM_X_ExtDrv;

	LSM6DSM_X_0_Data.comboData = &LSM6DSM_Combo_Data[0];
	ACCELERO_Data[LSM6DSM_X_0].pComponentData = (void *) &LSM6DSM_X_0_Data;
	ACCELERO_Data[LSM6DSM_X_0].pExtData = 0;

	*handle = (void *) &ACCELERO_SensorHandle[LSM6DSM_X_0];

	Sensor_IO_SPI_CS_Init(*handle);

	if (LSM6DSM_Combo_Data[0].isGyroInitialized == 0) {
		// SPI Serial Interface Mode selection --> 3Wires
		if (Sensor_IO_Write(*handle, LSM6DSM_ACC_GYRO_CTRL3_C, &data, 1)) {
			return COMPONENT_ERROR;
		}
	}

	driver = (ACCELERO_Drv_t *) ((DrvContextTypeDef *) (*handle))->pVTable;

	if (driver->Init == NULL) {
		memset((*handle), 0, sizeof(DrvContextTypeDef));
		*handle = NULL;
		return COMPONENT_ERROR;
	}

	if (driver->Init((DrvContextTypeDef *) (*handle)) == COMPONENT_ERROR) {
		memset((*handle), 0, sizeof(DrvContextTypeDef));
		*handle = NULL;
		return COMPONENT_ERROR;
	}

	/* Disable I2C interface */
	if (LSM6DSM_ACC_GYRO_W_I2C_DISABLE(*handle,
			LSM6DSM_ACC_GYRO_I2C_DISABLE_SPI_ONLY) == MEMS_ERROR) {
		return COMPONENT_ERROR;
	}

	/* Configure interrupt lines for LSM6DSM */
//	LSM6DSM_Sensor_IO_ITConfig();
	return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Enable(void *handle) {
	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;

	switch (ctx->spiDevice) {
	case LSM6DSM:
		HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port,
		SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_RESET);
		break;
	}
	return COMPONENT_OK;
}
uint8_t Sensor_IO_SPI_CS_Disable(void *handle) {
	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;

	switch (ctx->spiDevice) {
	case LSM6DSM:
		HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port,
		SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
		break;
	}
	return COMPONENT_OK;
}
uint8_t Sensor_IO_SPI_CS_Init(void *handle) {
	GPIO_InitTypeDef GPIO_InitStruct;
	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;

	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	switch (ctx->spiDevice) {
	case LSM6DSM:
		SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE()
		;
		GPIO_InitStruct.Pin = SENSORTILE_LSM6DSM_SPI_CS_Pin;
		/* Set the pin before init to avoid glitch */
		HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port,
		SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_Init(SENSORTILE_LSM6DSM_SPI_CS_Port, &GPIO_InitStruct);
		break;

	default:
		return COMPONENT_NOT_IMPLEMENTED;
	}
	return COMPONENT_OK;
}

/**
 * @brief Enable accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable(void *handle) {

	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;
	ACCELERO_Drv_t *driver = NULL;

	if (ctx == NULL) {
		return COMPONENT_ERROR;
	}

	driver = (ACCELERO_Drv_t *) ctx->pVTable;

	if (driver->Sensor_Enable == NULL) {
		return COMPONENT_ERROR;
	}

	if (driver->Sensor_Enable(ctx) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
void Accelero_Sensor_Handler(void *handle) {
	char dataOut[256];
//	uint8_t who_am_i;
//	float odr;
//	float fullScale;
	uint8_t id;
	SensorAxes_t acceleration;
	uint8_t status;
//	int32_t d1, d2;

	BSP_ACCELERO_Get_Instance(handle, &id);

	BSP_ACCELERO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR) {
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}
		sprintf(dataOut, "%d, %d, %d\r\n", (int) acceleration.AXIS_X,
				(int) acceleration.AXIS_Y, (int) acceleration.AXIS_Z);
		CDC_Transmit_FS((uint8_t*) dataOut, strlen(dataOut));
	}
}

/**
 * @brief Get the accelerometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Instance(void *handle, uint8_t *instance) {
	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;

	if (ctx == NULL) {
		return COMPONENT_ERROR;
	}

	if (instance == NULL) {
		return COMPONENT_ERROR;
	}

	*instance = ctx->instance;

	return COMPONENT_OK;
}

/**
 * @brief Check if the accelerometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_IsInitialized(void *handle, uint8_t *status) {
	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;

	if (ctx == NULL) {
		return COMPONENT_ERROR;
	}

	if (status == NULL) {
		return COMPONENT_ERROR;
	}

	*status = ctx->isInitialized;

	return COMPONENT_OK;
}

/**
 * @brief Get the accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written [mg]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_ACCELERO_Get_Axes(void *handle, SensorAxes_t *acceleration) {

	DrvContextTypeDef *ctx = (DrvContextTypeDef *) handle;
	ACCELERO_Drv_t *driver = NULL;

	if (ctx == NULL) {
		return COMPONENT_ERROR;
	}

	driver = (ACCELERO_Drv_t *) ctx->pVTable;

	if (acceleration == NULL) {
		return COMPONENT_ERROR;
	}

	if (driver->Get_Axes == NULL) {
		return COMPONENT_ERROR;
	}

	if (driver->Get_Axes(ctx, acceleration) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

/**
 * @brief  Configures sensor SPI interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_SPI_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	if (HAL_SPI_GetState(&SPI_Sensor_Handle) == HAL_SPI_STATE_RESET) {
		SENSORTILE_SENSORS_SPI_CLK_ENABLE()
		;
		SENSORTILE_SENSORS_SPI_GPIO_CLK_ENABLE()
		;

		GPIO_InitStruct.Pin = SENSORTILE_SENSORS_SPI_MOSI_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(SENSORTILE_SENSORS_SPI_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = SENSORTILE_SENSORS_SPI_SCK_Pin;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(SENSORTILE_SENSORS_SPI_Port, &GPIO_InitStruct);

		SPI_Sensor_Handle.Instance = SENSORTILE_SENSORS_SPI;
		SPI_Sensor_Handle.Init.Mode = SPI_MODE_MASTER;
		SPI_Sensor_Handle.Init.Direction = SPI_DIRECTION_1LINE;
		SPI_Sensor_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
		SPI_Sensor_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
		SPI_Sensor_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
		SPI_Sensor_Handle.Init.NSS = SPI_NSS_SOFT;
		SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 5 MHz
//    SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 2.5MHz
		SPI_Sensor_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
		SPI_Sensor_Handle.Init.TIMode = SPI_TIMODE_DISABLED;
		SPI_Sensor_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
		SPI_Sensor_Handle.Init.CRCPolynomial = 7;
		SPI_Sensor_Handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		SPI_Sensor_Handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
		HAL_SPI_Init(&SPI_Sensor_Handle);

		SPI_1LINE_TX(&SPI_Sensor_Handle);
		__HAL_SPI_ENABLE(&SPI_Sensor_Handle);
	}
	return COMPONENT_OK;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
