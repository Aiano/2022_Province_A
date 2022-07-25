/**
 * @file uart_parser.c
 * @brief 串口指令解析器
 * @details 指令格式 <命令字母> + <double数值>
 */

#include "uart_parser.h"
#include "string.h"
#include "stdlib.h"

#define RX_BUFFER_SIZE 64
// receive data byte by byte
// 逐Byte接收数据
uint8_t aRxBuffer = 0;
// and store aRxBuffer into RxBuffer
// 把aRxBuffer里的单Byte数据存到RxBuffer里
char RxBuffer[RX_BUFFER_SIZE];
int RxBufferCounts = 0;
// feedback message
// 返回给上位机的消息，表示收到命令
char FeedBackMessage[RX_BUFFER_SIZE + 20];

int call_count = 0;

char received_state;
int16_t received_value;

/**
 * @brief 初始化串口解析器
 * @param huart
 */
void uart_parser_init(UART_HandleTypeDef *huart) {
    // clear rx buffer
    // 清空接收缓冲数组
    memset(RxBuffer, 0, sizeof(RxBuffer));
    // enable uart interrupt once
    // 第一次启动中断，之后触发后需要再次调用该函数
    HAL_UART_Receive_IT(huart, (uint8_t *) &aRxBuffer, 1);
}


/**
 * @brief redefine Rx Transfer completed callbacks function.
 * @brief 重定义Rx Transfer completed callbacks function（这是HAL库的一个weak函数，在打开串口中断，并且接收串口数据完成时触发）
 * @param huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // when enter this function, a byte of data has been stored in aRxBuffer.
    // 当进入这个函数，说明已经有一个数据存储到aRxBuffer里面了
    // now, push aRxBuffer into RxBuffer.
    // 现在要做的事，就是把aRxBuffer存入RxBuffer
    RxBuffer[RxBufferCounts++] = aRxBuffer;
    // if encounter LF, the transfer process has completed.
    // 如果遇到了LF，说明传输结束，这也意味着上位机传来的命令必须带有 \n 字符
    // LF = Linefeed 换行 ASCII 10 \n newline
    // CR = Carriage Return 回车 ASCII 13 \r return
    if (RxBuffer[RxBufferCounts - 1] == 10) {
        // output feedback message
        // 输出返回信息给上位机
        memset(FeedBackMessage, 0, sizeof(FeedBackMessage));
        strcat(FeedBackMessage, "Received:");
        strcat(FeedBackMessage, RxBuffer);
        HAL_UART_Transmit(huart, FeedBackMessage, sizeof(FeedBackMessage), 0xFF);

        // do something to the RxBuffer
        // 开始对接收到的数据进行处理
        received_state = RxBuffer[0];
        received_value = atoi(RxBuffer + 1);

        // clear RxBuffer and reset counts
        memset(RxBuffer, 0, sizeof(RxBuffer));
        RxBufferCounts = 0;
    }
    // enable uart interrupt again to receive rest data, or for the next reception
    // 串口中断触发一次后，必须重新启动中断
    HAL_UART_Receive_IT(huart, (uint8_t *) &aRxBuffer, 1);
}