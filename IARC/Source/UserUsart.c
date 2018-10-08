#include "UserUsart.h"
#include "string.h"
#include <stdarg.h>

UART_HandleTypeDef *MainUsartSource;
volatile uint8_t Usart_Tx_NotCompleted_Flags[NumUsartSources + 1] = {0};
volatile uint8_t Usart_Rx_NotCompleted_Flags[NumUsartSources + 1] = {0};
static char Usart_Tx_Buffer[Usart_Tx_Buffer_Size];
static char Usart_Rx_Buffer[Usart_Rx_Buffer_Size];


int Usart_x(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) return 1;
	if(huart->Instance == USART2) return 2;
	if(huart->Instance == USART3) return 3;
	if(huart->Instance == UART4) return 4;
	if(huart->Instance == UART5) return 5;
	if(huart->Instance == USART6) return 6;
	return 0;
}
void SetMainUsartSource(UART_HandleTypeDef *huart) {
	MainUsartSource = huart;
}
void UsartSend(char* txBuff, uint16_t Size, UART_HandleTypeDef *huart) {
	HAL_UART_Transmit(huart, (uint8_t*)txBuff, Size, Tx_Time_Out);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	Usart_Tx_NotCompleted_Flags[Usart_x(huart)] = 0;
}
void UsartSend_DMA(char* txBuff, uint16_t Size, UART_HandleTypeDef *huart) {
	while(Usart_Tx_NotCompleted_Flags[Usart_x(huart)] == 1);
	Usart_Tx_NotCompleted_Flags[Usart_x(huart)] = 1;
	HAL_UART_Transmit_DMA(huart, (uint8_t*)txBuff, Size);
} 
int fprintf_dma(UART_HandleTypeDef *huart, const char*format, ...) {
	va_list args;
    int n;
    va_start(args, format);
    n = vsprintf(Usart_Tx_Buffer, format, args);
    va_end(args);
    UsartSend_DMA(Usart_Tx_Buffer, n, huart);
    return n;
}

int printf_dma(const char*format, ...) {
	va_list args;
    int n;
    va_start(args, format);
    n = vsprintf(Usart_Tx_Buffer, format, args);
    va_end(args);
    UsartSend_DMA(Usart_Tx_Buffer, n, MainUsartSource);
    return n;
}
int fputc(int c, FILE *f)  
{ 
	Usart_Tx_Buffer[0] = c;
	HAL_UART_Transmit(MainUsartSource, (uint8_t*)Usart_Tx_Buffer, 1, Tx_Time_Out);
    return c; 
} 

void UsartRead(char* RxBuff, uint16_t Size, UART_HandleTypeDef *huart) {
	HAL_UART_Receive(huart, (uint8_t*)RxBuff, Size, Rx_Time_Out);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	Usart_Rx_NotCompleted_Flags[Usart_x(huart)] = 0;
}
void UsartRead_DMA(char* RxBuff, uint16_t Size, UART_HandleTypeDef *huart) {
	while(Usart_Rx_NotCompleted_Flags[Usart_x(huart)] == 1);
	Usart_Rx_NotCompleted_Flags[Usart_x(huart)] = 1;
	HAL_UART_Receive_DMA(huart, (uint8_t*)RxBuff, Size);
}

int fgetc(FILE *f)
{
	//bug fixme
	UsartRead(Usart_Rx_Buffer, 1, MainUsartSource);
	return (int)Usart_Rx_Buffer[0];
}
void readLine(char* str) {
	int i = 0;
	str[i] = getchar();
	while(str[i++] != '\r') str[i] = getchar();
	str[i-1] = '\0';
}

