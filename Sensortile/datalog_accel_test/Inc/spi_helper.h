#include "stm32l4xx_hal.h"
#include "component.h"

extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr,
		uint8_t *pBuffer, uint16_t nBytesToWrite);
uint8_t Sensor_IO_SPI_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer,
		uint16_t nBytesToWrite);
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer,
		uint16_t nBytesToRead);
uint8_t Sensor_IO_SPI_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer,
		uint16_t nBytesToRead);
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val,
		uint16_t nBytesToRead);
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
