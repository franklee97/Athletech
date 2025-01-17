/*
 * ESP8266_HAL.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */

#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

#define wifi_uart &huart1
#define pc_uart &huart5

char buffer[20];

char *Basic_inclusion =
        "<!DOCTYPE html> <html>\n<head><meta name=\"viewport\"\
		content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n\
		<title>LED CONTROL</title>\n<style>html { font-family: Helvetica; \
		display: inline-block; margin: 0px auto; text-align: center;}\n\
		body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\
		h3 {color: #444444;margin-bottom: 50px;}\n.button {display: block;\
		width: 80px;background-color: #1abc9c;border: none;color: white;\
		padding: 13px 30px;text-decoration: none;font-size: 25px;\
		margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n\
		.button-on {background-color: #1abc9c;}\n.button-on:active \
		{background-color: #16a085;}\n.button-off {background-color: #34495e;}\n\
		.button-off:active {background-color: #2c3e50;}\np {font-size: 14px;color: #888;margin-bottom: 10px;}\n\
		</style>\n</head>\n<body>\n<h1>ESP8266 LED CONTROL</h1>\n";

char *LED_ON =
        "<p>LED Status: ON</p><a class=\"button button-off\" href=\"/ledoff\">OFF</a>";
char *LED_OFF =
        "<p>LED1 Status: OFF</p><a class=\"button button-on\" href=\"/ledon\">ON</a>";
char *Terminate = "</body></html>";

/*****************************************************************************************************************************************/

void ESP_Init(char *SSID, char *PASSWD) {
    char data[80];

    Ringbuf_init();

    Uart_sendstring("AT+RST\r\n", wifi_uart);
    Uart_sendstring("RESETTING.", pc_uart);
    for (int i = 0; i < 5; i++) {
        Uart_sendstring(".", pc_uart);
        HAL_Delay(1000);
    }
    HAL_Delay(1000);
    Uart_flush(wifi_uart);

    /********* AT **********/
    Uart_sendstring("AT\r\n", wifi_uart);
    HAL_Delay(1000);
    while (!(Wait_for("OK\r\n", wifi_uart)))
        ;
    Uart_sendstring("AT---->OK\r\n\n", pc_uart);
    Uart_flush(wifi_uart);

    /********* AT+CWMODE=1 **********/
    Uart_sendstring("AT+CWMODE=1\r\n", wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)))
        ;
    Uart_sendstring("CW MODE---->1\r\n\n", pc_uart);
    Uart_flush(wifi_uart);

    /********* AT+CWJAP="SSID","PASSWD" **********/
    Uart_sendstring("connecting... to the provided AP\r\n", pc_uart);
    sprintf(data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
    Uart_sendstring(data, wifi_uart);
    while (!(Wait_for("WIFI GOT IP\r\n", wifi_uart)))
        ;
    HAL_Delay(1000);
    sprintf(data, "Connected to,\"%s\"\r\n\n", SSID);
    Uart_sendstring(data, pc_uart);
    Uart_flush(wifi_uart);

    /********* AT+CIPMUX=0 **********/
    Uart_sendstring("AT+CIPMUX=0\r\n", wifi_uart);

    while (!(Wait_for("OK\r\n", wifi_uart)))
        ;
    HAL_Delay(2000);
    Uart_sendstring("Enabled single connection\r\n", pc_uart);
    Uart_flush(wifi_uart);

    /********* AT+CIPSTART=local server ********/
    Uart_sendstring("AT+CIPSTART=\"TCP\",\"192.168.43.224\",8080\r\n",
    wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)))
        ;
    Uart_sendstring("Connected to local server.\r\n", pc_uart);
    Uart_flush(wifi_uart);
    HAL_Delay(2000);

}

int Server_Send(char *str, int Link_ID) {
    int len = strlen(str);
    char data[80];
    sprintf(data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
    Uart_sendstring(data, wifi_uart);
    while (!(Wait_for(">", wifi_uart)))
        ;
    Uart_sendstring(str, wifi_uart);
    while (!(Wait_for("SEND OK", wifi_uart)))
        ;
    sprintf(data, "AT+CIPCLOSE=5\r\n");
    Uart_sendstring(data, wifi_uart);
    while (!(Wait_for("OK\r\n", wifi_uart)))
        ;
    return 1;
}

void Server_Handle(char *str, int Link_ID) {
    char datatosend[1024] = { 0 };
    if (!(strcmp(str, "/ledon"))) {
        sprintf(datatosend, Basic_inclusion);
        strcat(datatosend, LED_ON);
        strcat(datatosend, Terminate);
        Server_Send(datatosend, Link_ID);
    }

    else if (!(strcmp(str, "/ledoff"))) {
        sprintf(datatosend, Basic_inclusion);
        strcat(datatosend, LED_OFF);
        strcat(datatosend, Terminate);
        Server_Send(datatosend, Link_ID);
    }

    else {
        sprintf(datatosend, Basic_inclusion);
        strcat(datatosend, LED_OFF);
        strcat(datatosend, Terminate);
        Server_Send(datatosend, Link_ID);
    }

}

void Server_Start(void) {
    char buftocopyinto[64] = { 0 };
    char Link_ID;
    while (!(Get_after("+IPD,", 1, &Link_ID, wifi_uart)))
        ;
    Link_ID -= 48;
    while (!(Copy_upto(" HTTP/1.1", buftocopyinto, wifi_uart)))
        ;
    if (Look_for("/ledon", buftocopyinto) == 1) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, 1);
        Server_Handle("/ledon", Link_ID);
    }

    else if (Look_for("/ledoff", buftocopyinto) == 1) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, 0);
        Server_Handle("/ledoff", Link_ID);
    }

    else if (Look_for("/favicon.ico", buftocopyinto) == 1)
        ;

    else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, 0);
        Server_Handle("/ ", Link_ID);
    }
}

void bufclr(char *buf) {
    int len = strlen(buf);
    for (int i = 0; i < len; i++)
        buf[i] = '\0';
}

void ESP_Send_Multi(char *APIkey, int numberoffileds, uint16_t value[]) {
    char local_buf[500] = { 0 };
    char local_buf2[30] = { 0 };
    char field_buf[200] = { 0 };

//    sprintf(local_buf, "GET /update?api_key=%s", APIkey);
//    for (int i = 0; i < numberoffileds; i++) {
    sprintf(field_buf, "EMG:%u,FSR:%u,IMU:%u,TS:%u", value[0], value[1],
            value[2], value[3]);
    strcat(local_buf, field_buf);
//    }

//    strcat(local_buf, "\r\n");
    int len = strlen(local_buf);

    sprintf(local_buf2, "AT+CIPSEND=%d\r\n", len);
    Uart_sendstring(local_buf2, wifi_uart);
    while (!(Wait_for(">", wifi_uart)))
        ;
//    HAL_Delay(2000);
    Uart_flush(wifi_uart);

    Uart_sendstring(local_buf, wifi_uart);
    while (!(Wait_for("SEND OK\r\n", wifi_uart)))
        ;
    Uart_flush(wifi_uart);
//    while (!(Wait_for("CLOSED")))
//        ;

    bufclr(local_buf);
    bufclr(local_buf2);

    Ringbuf_init();
    HAL_Delay(100);

}
