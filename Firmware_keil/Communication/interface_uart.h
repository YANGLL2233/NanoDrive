#ifndef INTERFACE_UART_H
#define INTERFACE_UART_H

#include <stdarg.h>
#include <stdio.h>

void usart_printf(const char *format, ...);
void usart_intrface_init(void);

#endif
