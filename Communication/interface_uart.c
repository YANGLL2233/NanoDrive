#include "interface_uart.h"

#include "main.h"
#include <stdlib.h>

#include "open_loop_controller.h"
#include "drv8323.h"

extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;

#define TX_BUFFER_LEN 512
#define RX_BUFFER_LEN 128

uint8_t rx_buffer[RX_BUFFER_LEN];

extern OpenLoopController_t open_loop_controller;

void usart_printf(const char *format, ...)
{
    char buffer[TX_BUFFER_LEN];
    va_list args;
    va_start(args, format);
    int32_t count = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (count > 0 && count < TX_BUFFER_LEN) {
        //
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buffer, count);
    }
}

void usart_intrface_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUFFER_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void uart_data_handle(void)
{
    switch (rx_buffer[0]) {
        case 'A': {
            drv8323_enable();
            open_loop_controller.target_vel_     = 62.8f;
            open_loop_controller.target_voltage_ = 0.5f;
            break;
        }
        case 'C': {
            clear_error(ERROR_ALL);
            break;
        }

        case 'D': {
            drv8323_disable();
            open_loop_controller.target_vel_     = 0;
            open_loop_controller.target_voltage_ = 0;
            break;
        }
        case 'T': {
            open_loop_controller.target_vel_ = strtof((char *)rx_buffer + 1, NULL);
            break;
        }

        case 'M': {
            open_loop_controller.target_voltage_ = strtof((char *)rx_buffer + 1, NULL);
            break;
        }

        default: {
            drv8323_disable();
            break;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3) {
        if (Size <= RX_BUFFER_LEN) {
            //
            uart_data_handle();
            memset(rx_buffer, 0, Size);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUFFER_LEN);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}
