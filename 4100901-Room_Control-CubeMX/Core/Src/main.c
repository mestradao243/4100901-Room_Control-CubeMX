
/* USER CODE BEGIN Header */
/**
 **********
 * @file           : main.c
 * @brief          : Main program body
 **********
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 **********
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_driver.h"    // Librería para controlar el LED
#include "ring_buffer.h"   // Librería para manejar un buffer circular
#include "keypad_driver.h" // Librería del teclado matricial (4x4)
#include <stdio.h>         // Para usar printf() y enviar mensajes por UART
#include <string.h>        // Para comparar cadenas y limpiar variables
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- CONFIGURACION DEL SISTEMA ---
/* USER CODE END PD */
#define PASSWORD "123A"          // Contraseña de 4 dígitos que el usuario debe ingresar
#define PASSWORD_LEN 4           // Longitud de la contraseña
#define DEBOUNCE_TIME_MS 200     // Tiempo mínimo entre pulsaciones para evitar rebotes
#define SUCCESS_LED_TIME_MS 4000 // Tiempo que el LED permanece encendido al ingresar la contraseña correcta

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // Estructura que guarda toda la configuración y el estado del puerto UART2 (comunicación serial)

/* USER CODE BEGIN PV */
// --- HANDLES Y BUFFERS ---
led_handle_t led1 = {.port = GPIOA, .pin = GPIO_PIN_5}; // LED del usuario (LD2) en el pin PA5

// Estructura que define los pines usados por el teclado matricial 4x4

// --- CONFIGURACIÓN DE HARDWARE DEL TECLADO 4x4 ---
// Filas:
//   R1 → PA10 (GPIOA pin 10)
//   R2 → PB3  (GPIOB pin 3)
//   R3 → PB5  (GPIOB pin 5)
//   R4 → PB4  (GPIOB pin 4)
// Columnas:
//   C1 → PB10 (GPIOB pin 10) - Entrada con interrupción EXTI
//   C2 → PA8  (GPIOA pin 8)  - Entrada con interrupción EXTI
//   C3 → PA9  (GPIOA pin 9)  - Entrada con interrupción EXTI
//   C4 → PC7  (GPIOC pin 7)  - Entrada con interrupción EXTI

keypad_handle_t keypad = {
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port},
    .row_pins = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port},
    .col_pins = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}};

// --- BUFFER CIRCULAR PARA TECLAS ---
#define KEYPAD_BUFFER_LEN 16              // Tamaño máximo del buffer de teclas
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN]; // Arreglo donde se almacenan las teclas presionadas
ring_buffer_t keypad_rb;                  // Estructura de control del buffer circular

// --- VARIABLES DE CONTROL DE ACCESO ---
char entered_password[PASSWORD_LEN + 1] = {0}; // Arreglo para guardar los dígitos ingresados
uint8_t password_index = 0;                    // Índice que indica cuántos dígitos se han ingresado

// --- VARIABLES DE ESTADO PARA LÓGICA NO BLOQUEANTE ---
uint32_t last_key_press_time = 0; // Guarda el tiempo del último dígito leído (para evitar rebotes)
uint32_t led_timer_start = 0;     // Momento en el que el LED fue encendido
uint32_t led_on_duration = 0;     // Tiempo que el LED debe permanecer encendido
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// --- Prototipos de funciones internas ---
void SystemClock_Config(void);         // Configura la frecuencia y el reloj principal del microcontrolador
static void MX_GPIO_Init(void);        // Configura los pines GPIO (entradas, salidas e interrupciones)
static void MX_USART2_UART_Init(void); // Inicializa la UART2 para comunicación serial

/* USER CODE BEGIN PFP */
// --- FUNCIONES PERSONALIZADAS ---
// void process_key(uint8_t key); // (No usada en este código) - sería para procesar teclas recibidas
void manage_led_timer(void); // Controla el tiempo de encendido del LED (sin bloquear el programa)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Callback de la interrupción externa GPIO.
 * @note   Esta función se llama cuando se detecta un flanco en un pin configurado para interrupción.
 *         Se mantiene muy rápida: solo lee el teclado y guarda la tecla en un buffer.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  char key = keypad_scan(&keypad, GPIO_Pin); // Escanea el teclado para identificar qué tecla se presionó
  if (key != '\0')                           // Si se detectó una tecla válida
  {
    ring_buffer_write(&keypad_rb, (uint8_t)key); // Guarda la tecla en el buffer circular
  }
}

/**
 * @brief Procesa una tecla recibida del buffer del keypad.
 * @note  Contiene la lógica principal de la aplicación: feedback visual,
 *        almacenamiento de la contraseña y verificación.
 * @param key La tecla presionada a procesar.
 */
void lectura_de_clave(uint8_t llave)
{
  // --- Almacenar dígito presionado ---
  if (password_index < PASSWORD_LEN) // Si aún no se ingresaron las 4 teclas
  {
    entered_password[password_index++] = (char)llave; // Guarda el carácter en el arreglo
    printf("Digito presionado: %c\r\n", llave);       // Muestra por UART la tecla presionada
  }

  // --- Verificar contraseña cuando se ingresan los 4 dígitos ---
  if (password_index == PASSWORD_LEN)
  {
    if (strncmp(entered_password, PASSWORD, PASSWORD_LEN) == 0) // Compara con la contraseña correcta
    {
      printf("Contraseña correcta. ACCESO AUTORIZADO.\r\n"); // Mensaje de éxito
      // Encender LED por éxito de contraseña
      led_on(&led1);                         // Enciende el LED
      led_timer_start = HAL_GetTick();       // Guarda el momento de encendido
      led_on_duration = SUCCESS_LED_TIME_MS; // Mantener encendido 4 segundos
    }
    else
    {
      printf("Contraseña incorrecta. ACCESO DENEGADO.\r\n"); // Mensaje de error
      led_off(&led1);                                        // Asegura que el LED quede apagado si la clave es incorrecta
    }

    // --- Reiniciar para una nueva entrada ---
    password_index = 0;                                    // Reinicia el contador
    memset(entered_password, 0, sizeof(entered_password)); // Limpia la clave ingresada
    printf("\nSistema de acceso listo. Ingrese la contraseña de 4 digitos.\r\n");
  }
}

/**
 * @brief Gestiona el apagado automático del LED sin bloquear el programa.
 * @note  Esta función debe ser llamada repetidamente en el bucle principal.
 */

void manage_led_timer(void)
{
  // Si el LED está encendido y ya pasó el tiempo programado
  if (led_timer_start != 0 && (HAL_GetTick() - led_timer_start > led_on_duration))
  {
    led_off(&led1);      // Apaga el LED
    led_timer_start = 0; // Reinicia el temporizador
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  // Inicializa todos los periféricos configurados (GPIO, UART, etc.)
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // --- Inicialización de drivers personalizados ---
  led_init(&led1);                                                // Inicializa el LED conectado a PA5 (LD2)
  ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN); // Inicializa el buffer circular para almacenar teclas
  keypad_init(&keypad);                                           // Coloca todas las filas del teclado en bajo

  printf("Sistema de Control de Acceso Iniciado.\r\n"); // Mensaje inicial por UART
  printf("Ingrese la contraseña de 4 dígitos...\r\n");  // Solicitud al usuario
                                                        /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) // --- Bucle principal del programa ---
  {
    uint8_t key_from_buffer; // Variable para guardar la tecla leída desde el buffer

    // --- Lectura no bloqueante del teclado ---
    if (ring_buffer_read(&keypad_rb, &key_from_buffer)) // Si hay una tecla disponible en el buffer
    {
      uint32_t now = HAL_GetTick(); // Obtiene el tiempo actual en milisegundos

      // --- Control de rebote por software ---
      if (now - last_key_press_time > DEBOUNCE_TIME_MS) // Si ha pasado suficiente tiempo desde la última tecla
      {
        last_key_press_time = now;         // Actualiza el tiempo de la última pulsación
        lectura_de_clave(key_from_buffer); // Procesa la tecla (guarda y valida contraseña)
      }
    }

    // --- Control del LED sin bloquear el programa ---
    manage_led_timer(); // Apaga el LED automáticamente cuando pasa el tiempo definido

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // El bucle continúa ejecutándose indefinidamente
  }
  /* USER CODE END 3 */
}
/* ---------------------- FUNCIONES DE HARDWARE ---------------------- */

/**
 * @brief System Clock Configuration
 * @retval None
 * * Esta función define cómo el microcontrolador genera y distribuye las señales de reloj
 * que sincronizan todos los periféricos internos (CPU, buses, temporizadores, UART, GPIO, etc.).
 *
 * En este proyecto:
 * - Se usa el **oscilador interno HSI (High-Speed Internal)** de 16 MHz como fuente base.
 * - El HSI alimenta el **PLL (Phase-Locked Loop)**, que multiplica la frecuencia para
 *   obtener una velocidad de trabajo más alta.
 * - El PLL genera una **frecuencia del sistema (SYSCLK)** de 80 MHz aprox.
 * - Esta frecuencia se distribuye a los buses:
 *   - **AHB**: núcleo principal (CPU, memoria, DMA)
 *   - **APB1 y APB2**: periféricos como UART, GPIO, TIM, etc.
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN USART2_Init 2 */
#ifdef GNUC
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin | KEYPAD_R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEYPAD_R2_Pin | KEYPAD_R4_Pin | KEYPAD_R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin KEYPAD_R1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin | KEYPAD_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C1_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C4_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_C2_Pin KEYPAD_C3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C2_Pin | KEYPAD_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_R2_Pin KEYPAD_R4_Pin KEYPAD_R3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_R2_Pin | KEYPAD_R4_Pin | KEYPAD_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* MODIFICADO: Se cambia la prioridad de 0 (máxima) a 5 (media-baja).
   * Un valor numérico más bajo significa una prioridad más alta en ARM Cortex-M.
   * Usar prioridad 0 es arriesgado, ya que puede bloquear interrupciones del sistema.
   * Una prioridad de 5 es segura para periféricos de usuario como un teclado.
   */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Sobrescribir la función _write para redirigir printf a la UART
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
