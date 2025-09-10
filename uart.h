#ifndef UART_H
#define UART_H

#include <stdint.h>

// Definitions for each of the LED colors
#define RED 22
#define GREEN 26
#define BLUE 21

void init_uart(void);
void init_button(void);
void put(char *ptr_str);
uint8_t uart_getchar(void);
void uart_putchar(char ch);

#endif /* ifndef UART_H */
