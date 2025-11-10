#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

#include "main.h"
#include <stdint.h>

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

typedef struct // Estructura para manejar el keypad
{
    GPIO_TypeDef *row_ports[KEYPAD_ROWS];
    uint16_t row_pins[KEYPAD_ROWS];
    GPIO_TypeDef *col_ports[KEYPAD_COLS];
    uint16_t col_pins[KEYPAD_COLS];
} keypad_handle_t;

void keypad_init(keypad_handle_t *keypad);
char keypad_scan(keypad_handle_t *keypad, uint16_t col_pin);

#endif // KEYPAD_DRIVER_H