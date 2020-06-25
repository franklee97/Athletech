#include "spi_helper.h"
#include "main.h"

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer,
		uint16_t nBytesToWrite) {
	return Sensor_IO_SPI_Write(handle, WriteAddr, pBuffer, nBytesToWrite);
}

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer,
		uint16_t nBytesToWrite) {
	uint8_t i;

// Select the correct device
	Sensor_IO_SPI_CS_Enable(handle);

	SPI_Write(&SPI_Sensor_Handle, WriteAddr);

	for (i = 0; i < nBytesToWrite; i++) {
		SPI_Write(&SPI_Sensor_Handle, pBuffer[i]);
	}
// Deselect the device
	Sensor_IO_SPI_CS_Disable(handle);

	return COMPONENT_OK;
}

/**
 * @brief  This function writes a single byte on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @retval None
 */
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val) {
	/* check TXE flag */
	while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE)
		;

	/* Write the data */
	*((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;

	/* Wait BSY flag */
	while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY)
		;
	while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY)
		;
}

uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer,
		uint16_t nBytesToRead) {

	return Sensor_IO_SPI_Read(handle, ReadAddr, pBuffer, nBytesToRead);

}

/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer,
		uint16_t nBytesToRead) {
	/* Select the correct device */
	Sensor_IO_SPI_CS_Enable(handle);

	/* Write Reg Address */
	SPI_Write(&SPI_Sensor_Handle, ReadAddr | 0x80);

	/* Disable the SPI and change the data line to input */
	__HAL_SPI_DISABLE(&SPI_Sensor_Handle);
	SPI_1LINE_RX(&SPI_Sensor_Handle);

	/* Check if we need to read one byte or more */
	if (nBytesToRead > 1U) {
		SPI_Read_nBytes(&SPI_Sensor_Handle, pBuffer, nBytesToRead);
	} else {
		SPI_Read(&SPI_Sensor_Handle, pBuffer);
	}

	/* Deselect the device */
	Sensor_IO_SPI_CS_Disable(handle);

	/* Change the data line to output and enable the SPI */
	SPI_1LINE_TX(&SPI_Sensor_Handle);
	__HAL_SPI_ENABLE(&SPI_Sensor_Handle);

	return COMPONENT_OK;
}

/**
 * @brief  This function reads multiple bytes on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @param  nBytesToRead: number of bytes to read.
 * @retval None
 */
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val,
		uint16_t nBytesToRead) {
	/* Interrupts should be disabled during this operation */
	__disable_irq();
	__HAL_SPI_ENABLE(xSpiHandle);

	/* Transfer loop */
	while (nBytesToRead > 1U) {
		/* Check the RXNE flag */
		if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE) {
			/* read the received data */
			*val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
			val += sizeof(uint8_t);
			nBytesToRead--;
		}
	}
	/* In master RX mode the clock is automaticaly generated on the SPI enable.
	 So to guarantee the clock generation for only one data, the clock must be
	 disabled after the first bit and before the latest bit of the last Byte received */
	/* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */

	__DSB();
	__DSB();
	__HAL_SPI_DISABLE(xSpiHandle);

	__enable_irq();

	while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
		;
	/* read the received data */
	*val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
	while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY)
		;
}

/**
 * @brief  This function reads a single byte on SPI 3-wire.
 * @param  xSpiHandle : SPI Handler.
 * @param  val : value.
 * @retval None
 */
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val) {
	/* In master RX mode the clock is automaticaly generated on the SPI enable.
	 So to guarantee the clock generation for only one data, the clock must be
	 disabled after the first bit and before the latest bit */
	/* Interrupts should be disabled during this operation */

	__disable_irq();

	__HAL_SPI_ENABLE(xSpiHandle);
	__asm("dsb\n");
	__asm("dsb\n");
	__HAL_SPI_DISABLE(xSpiHandle);

	__enable_irq();

	while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
		;
	/* read the received data */
	*val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
	while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY)
		;
}

