/*
 * ESPDataLogger.h
 *
 *  Created on: May 26, 2020
 *      Author: controllerstech
 */

#ifndef INC_ESPDATALOGGER_H_
#define INC_ESPDATALOGGER_H_

/**
 * Initializes Wifi connection using SSID and PASSWD
 * and connects to a proxy server with IP address Server_IP
 */
void ESP_Init(char *SSID, char *PASSWD, char *Server_IP);
void ESP_Send_Multi(int32_t value[]);
void Uart_debug_sendstring(char* input);
#endif /* INC_ESPDATALOGGER_H_ */
