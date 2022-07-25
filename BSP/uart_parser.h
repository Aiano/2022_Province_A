/**
 * @file uart_parser.c
 * @brief 串口指令解析器
 * @details 指令格式 <命令字母> + <double数值>
 */

#ifndef HAL_UART_PARSER_UART_PARSER_H
#define HAL_UART_PARSER_UART_PARSER_H

#include "usart.h"

#ifdef __cplusplus
extern "C"{
#endif

extern char received_state;
extern int16_t received_value;

void uart_parser_init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif //HAL_UART_PARSER_UART_PARSER_H
