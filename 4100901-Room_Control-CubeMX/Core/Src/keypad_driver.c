#include "keypad_driver.h"
#include "main.h"

// Mapa estándar de teclas (4x4)
static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

/**
 * @brief Inicializa las filas en nivel bajo.
 * Esto deja el teclado listo para detectar flancos descendentes por columna.
 */
void keypad_init(keypad_handle_t *keypad) // iniciar keypad paras que las filas esten en bajo
{
    for (int i = 0; i < KEYPAD_ROWS; i++) // recorrer filas
    {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // poner en bajo las filas
    }
}

/**
 * @brief Escanea qué tecla fue presionada en función de la columna activada.
 */
char keypad_scan(keypad_handle_t *keypad, uint16_t col_pin) // escanea la tecla presionada
{
    HAL_Delay(5); // Delay de antirrebote

    // Determinar qué columna generó la interrupción
    int col_index = -1;
    for (int i = 0; i < KEYPAD_COLS; i++) // recorrer columnas
    {
        if (keypad->col_pins[i] == col_pin) // si la columna es igual al pin de la columna, guarda el índice
        {
            col_index = i;
            break;
        }
    }
    if (col_index == -1) // si no se encontró la columna
        return '\0';     // Columna inválida

    char key_pressed = '\0';

    // Poner todas las filas en ALTO antes de escanear
    for (int i = 0; i < KEYPAD_ROWS; i++)
    {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET); // poner en alto las filas
    }

    // Activar una fila a la vez
    for (int row = 0; row < KEYPAD_ROWS; row++) // recorrer filas
    {
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_RESET);
        HAL_Delay(1);

        // Si la columna sigue en bajo → tecla encontrada
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET)
        {
            key_pressed = keypad_map[row][col_index];

            // Esperar a que se suelte la tecla
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET)
                ;
            break;
        }

        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET); // Restaurar fila en ALTO
    }

    // Restaurar filas en bajo
    keypad_init(keypad);
    return key_pressed;
}