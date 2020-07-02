/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "LSM303AGR_MAG_driver_HL.h"
#include "LSM303AGR_MAG_driver.h"
#include "LSM6DSM_ACC_GYRO_driver_HL.h"
#include "LSM6DSM_ACC_GYRO_driver.h"
#include "spi_helper.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr,
		uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t WriteAddr, uint8_t *pBuffer,
		uint16_t nBytesToWrite);

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define LSM6DSM_ACC_GYRO_WHO_AM_I         0x6A
#define LSM6DSM_ACC_GYRO_I2C_ADDRESS_HIGH  0xD6  // SAD[0] = 1

#define LSM303AGR_MAG_WHO_AM_I         0x40


#define SENSORTILE_LSM6DSM_SPI_CS_Port	          GPIOB
#define SENSORTILE_LSM6DSM_SPI_CS_Pin     	  GPIO_PIN_12
#define SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define SENSORTILE_SENSORS_SPI                    SPI2

#define SENSORTILE_LSM303AGR_M_SPI_CS_Port	  GPIOB
#define SENSORTILE_LSM303AGR_M_SPI_CS_Pin     	  GPIO_PIN_1
#define SENSORTILE_LSM303AGR_M_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define SENSORTILE_SENSORS_SPI_Port               GPIOB
#define SENSORTILE_SENSORS_SPI_MOSI_Pin           GPIO_PIN_15
#define SENSORTILE_SENSORS_SPI_SCK_Pin            GPIO_PIN_13

#define SENSORTILE_SENSORS_SPI_CLK_ENABLE()       __SPI2_CLK_ENABLE()
#define SENSORTILE_SENSORS_SPI_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

/* USER CODE BEGIN PTD */
typedef enum {
	ACCELERO_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
	LSM6DSM_X_0, /* . */
	LSM303AGR_X_0 /* . */
} ACCELERO_ID_t;

typedef enum {
	GYRO_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
	LSM6DSM_G_0, /* Default on board. */
} GYRO_ID_t;

typedef enum
{
  MAGNETO_SENSORS_AUTO = -1,     /* Always first element and equal to -1 */
  LSM303AGR_M_0                      /* Default on board. */
} MAGNETO_ID_t;


typedef enum {
//  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
	LSM6DSM = 0, /* LSM6DSM. */
	LSM303AGR_X, /* LSM303AGR Accelerometer */
	LSM303AGR_M, /* LSM303AGR Magnetometer */
	LPS22HB /* LPS22HB */
} SPI_Device_t;

extern SPI_HandleTypeDef SPI_Sensor_Handle;

DrvStatusTypeDef BSP_ACCELERO_Init(ACCELERO_ID_t id, void **handle);
DrvStatusTypeDef BSP_LSM6DSM_ACCELERO_Init(void **handle);
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable(void *handle);
SensorAxes_t Accelero_Sensor_Handler(void *handle);
DrvStatusTypeDef BSP_ACCELERO_Get_Instance(void *handle, uint8_t *instance);
DrvStatusTypeDef BSP_ACCELERO_IsInitialized(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_ACCELERO_Get_Axes(void *handle, SensorAxes_t *acceleration);

DrvStatusTypeDef BSP_GYRO_Init(GYRO_ID_t id, void **handle);
DrvStatusTypeDef BSP_LSM6DSM_GYRO_Init(void **handle);
DrvStatusTypeDef BSP_GYRO_Sensor_Enable(void *handle);
SensorAxes_t Gyro_Sensor_Handler(void *handle);
DrvStatusTypeDef BSP_GYRO_IsInitialized(void *handle, uint8_t *status);
DrvStatusTypeDef BSP_GYRO_Get_Instance(void *handle, uint8_t *instance);
DrvStatusTypeDef BSP_GYRO_Get_Axes(void *handle, SensorAxes_t *angular_velocity);

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle );
DrvStatusTypeDef BSP_LSM303AGR_MAGNETO_Init( void **handle );
SensorAxes_t Magneto_Sensor_Handler( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field );



uint8_t Sensor_IO_SPI_CS_Init(void *handle);
DrvStatusTypeDef Sensor_IO_SPI_Init(void);
uint8_t Sensor_IO_SPI_CS_Enable(void *handle);
uint8_t Sensor_IO_SPI_CS_Disable(void *handle);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
