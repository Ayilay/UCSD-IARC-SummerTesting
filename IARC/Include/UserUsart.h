#ifndef __USERUSART_H
#define __USERUSART_H

#include "stm32f4xx_hal.h"

#define NumUsartSources 6
#define Usart_Tx_Buffer_Size 100
#define Usart_Rx_Buffer_Size 100
#define Tx_Time_Out 0xffff
#define Rx_Time_Out 0xffff


void SetMainUsartSource(UART_HandleTypeDef *huart);
void UsartSend(char* txBuff, uint16_t Size, UART_HandleTypeDef *huart);
void UsartSend_DMA(char* txBuff, uint16_t Size, UART_HandleTypeDef *huart);
int fprintf_dma(UART_HandleTypeDef *huart, const char*format, ...);
int printf_dma(const char*format, ...);

void UsartRead(char* RxBuff, uint16_t Size, UART_HandleTypeDef *huart);
void UsartRead_DMA(char* RxBuff, uint16_t Size, UART_HandleTypeDef *huart);
void readLine(char* str);
#endif
