/*
 * ESP8266_HAL.h
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */

#ifndef INC_ESP8266_HAL_H_
#define INC_ESP8266_HAL_H_


void ESP_Init (char *SSID, char *PASSWD);

void Server_Start (void);
void ESP_Send_Multi(char *APIkey, int numberoffileds, uint16_t value[]);

#endif /* INC_ESP8266_HAL_H_ */
