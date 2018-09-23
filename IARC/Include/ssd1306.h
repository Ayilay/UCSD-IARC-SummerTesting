#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "fonts.h"

//Originally from Olivier Van den Eede 2016
//Modified by Le Tan Phuc to be suitable for SSD1306 0.91" OLED display
//Work with STM32Cube MX HAL library
//Jul 2017

#ifndef SSD1306_H
#define SSD1306_H

#define SSD1306_I2C_PORT    hi2c1 // The HAL I2C HandleTypeDef used for this screen

#define SSD1306_I2C_ADDR    0x78  // I2C Address of device
#define SSD1306_WIDTH       128    // Dimensions in Pixels
#define SSD1306_HEIGHT      32



typedef enum {
  Black = 0x00, /*!< Black color, no pixel */
  White = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR;


typedef struct {
  uint16_t CurrentX;
  uint16_t CurrentY;
  uint8_t Inverted;
  uint8_t Initialized;
} SSD1306_t;

uint8_t SSD1306_Init(void);
void SSD1306_Fill(SSD1306_COLOR color);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char SSD1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char SSD1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void SSD1306_SetCursor(uint8_t x, uint8_t y);

void SSD1306_WriteCommand(uint8_t command);

#endif
