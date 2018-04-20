#include "SendMessage.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "string.h"

extern UART_HandleTypeDef huart2;

void sendStaicMessage(char * text)
{
    if (text[0]!='\0')
    {
        HAL_UART_Transmit(&huart2, (char *) text, strlen(text), HAL_MAX_DELAY);
    }
}

void sendCharter(char character)
{
    char text[2];
    text[0] = character;
    text[1] = '\n';
    HAL_UART_Transmit(&huart2, (char *) text, strlen(text), HAL_MAX_DELAY);
}
