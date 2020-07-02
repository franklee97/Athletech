/*
 * ESPDataLogger.c
 *
 *  Created on: May 26, 2020
 *      Author: controllerstech
 */

#include "UartRingbuffer.h"
#include "ESPDataLogger.h"
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void bufclr(char *buf) {
	int len = strlen(buf);
	for (int i = 0; i < len; i++)
		buf[i] = '\0';
}

void ESP_Init(char *SSID, char *PASSWD, char *Server_IP) {
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n");
	Uart_debug_sendstring("Resetting...\r\n\r\n");
	HAL_Delay(1000);

	Uart_flush();

	/********* AT **********/
	Uart_sendstring("AT\r\n");
	Uart_debug_sendstring("AT--->");
	HAL_Delay(1000);
	while (!(Wait_for("OK\r\n")))
		;
	Uart_debug_sendstring("OK\r\n\r\n");

	Uart_flush();

	/********* AT+CWMODE=1 **********/
	Uart_sendstring("AT+CWMODE=1\r\n");
	Uart_debug_sendstring("Setting Wifi mode as client--->");
	while (!(Wait_for("OK\r\n")))
		;
	Uart_debug_sendstring("OK\r\n\r\n");

	Uart_flush();

	/********* AT+CWJAP="SSID","PASSWD" **********/
	sprintf(data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_debug_sendstring("Connecting to Wifi--->");
	Uart_sendstring(data);
	while (!(Wait_for("GOT IP\r\n")))
		;
	sprintf(data, "\"%s\"OK\r\n\r\n", SSID);
	Uart_debug_sendstring(data);

	Uart_flush();

	/********* AT+CIPMUX=0 **********/
	Uart_sendstring("AT+CIPMUX=0\r\n");
	Uart_debug_sendstring("Setting single connection--->");
	while (!(Wait_for("OK\r\n")))
		;
	Uart_debug_sendstring("OK\r\n\r\n");
	sprintf(data, "AT+CIPSTART=\"TCP\",\"%s\",8080\r\n", Server_IP);
	Uart_sendstring(data);
	sprintf(data, "Connecting to local server hosted on %s--->", Server_IP);
	Uart_debug_sendstring(data);
	while (!(Wait_for("OK\r\n")))
		;
	Uart_debug_sendstring("OK\r\n\r\n");
	Uart_flush();

}

void ESP_Send_Multi(int32_t value[]) {
	char data[80];

	char local_buf[700] = { 0 };        // AT+CIPSEND=n
	char field_buf[700] = { 0 };        // Actual data

	sprintf(field_buf,
			"EMG:%ld,IMU-X:%ld,IMU-Y:%ld,IMU-Z:%ld,G-X:%ld,G-Y:%ld,G-Z:%ld,M-X:%ld,M-Y:%ld,M-X:%ld",
			value[0], value[2], value[3], value[4], value[5],
			value[6], value[7],value[8],value[9],value[10]);
	int len = strlen(field_buf);


	sprintf(local_buf, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf);
	while (!(Wait_for(">")))
		;

	Uart_sendstring(field_buf);
	while (!(Wait_for("SEND OK\r\n")))
		;

//	sprintf(data, "Sent the following data: EMG:%u,FSR:%u,IMU:%u,TS:%u\r\n",
//			value[0], value[1], value[2], value[3]);
	//sprintf(data, "%u\r\n", value[0]);
	//Uart_debug_sendstring(data);

	bufclr(field_buf);
	bufclr(local_buf);
	Ringbuf_init();

}

void Uart_debug_sendstring(char* input) {
	CDC_Transmit_FS((uint8_t*) input, strlen((char*) input));
}
