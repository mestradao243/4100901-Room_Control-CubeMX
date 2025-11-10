#include "ring_buffer.h"

/**
 * función: Inicializa la estructura del buffer circular.
 *
 * rb        Puntero a la estructura ring_buffer_t.
 * buffer    Puntero al arreglo de memoria que almacenará los datos.
 * capacity  Tamaño total del buffer (número de elementos).
 *
 * Esta función no reserva memoria, solo asocia el buffer ya existente
 * con la estructura y resetea los punteros de cabeza y cola.
 */
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity)
{
    rb->buffer = buffer;     // Se asigna el arreglo de almacenamiento
    rb->capacity = capacity; // Se define la capacidad máxima
    rb->head = 0;            // Índice de escritura (posición inicial)
    rb->tail = 0;            // Índice de lectura (posición inicial)
    rb->is_full = false;     // Estado inicial: no está lleno
}

/**
 * función: Escribe un byte en el buffer circular.
 *
 * rb    Puntero al ring buffer.
 * data  Dato a escribir.
 * true  Si el byte fue escrito correctamente.
 * false Si el buffer está lleno y no se puede escribir.
 *
 * La función calcula la siguiente posición del puntero de escritura (head).
 * Si esa posición coincide con el puntero de lectura (tail),
 * significa que el buffer está lleno y no se escribe el dato.
 */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    if (ring_buffer_is_full(rb))
    {
        // sobre escribe el dato más viejo
        rb->tail = (rb->tail + 1) % rb->capacity;
    }
    rb->buffer[rb->head] = data;              // Escribe el dato en la posición actual de head
    rb->head = (rb->head + 1) % rb->capacity; // Avanza head de manera circular
    return true;                              // Escritura exitosa
}

/**
 * función: Lee un byte del buffer circular.
 *
 * rb    Puntero al ring buffer.
 * data  Puntero donde se almacenará el dato leído.
 * true  Si se leyó un dato correctamente.
 * false Si el buffer está vacío.
 *
 * Si los punteros head y tail son iguales, el buffer está vacío y no hay datos por leer.
 */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    if (ring_buffer_is_empty(rb))
    {
        return false; // No hay datos para leer
    }
    *data = rb->buffer[rb->tail];             // Lee el dato en la posición actual de tail
    rb->tail = (rb->tail + 1) % rb->capacity; // Avanza tail de manera circular
    return true;                              // Lectura exitosa
}

/**
 * función: Devuelve la cantidad de elementos almacenados en el buffer.
 *
 * rb  Puntero al ring buffer.
 * Número de bytes actualmente en el buffer.
 *
 * El cálculo considera el caso en que el puntero de escritura (head)
 * haya pasado por el final del buffer y vuelto al inicio.
 */
uint16_t ring_buffer_count(ring_buffer_t *rb)
{
    if (rb->is_full)
    {
        return rb->capacity;
    }
    if (rb->head >= rb->tail)
    {
        return rb->head - rb->tail; // head está adelante de tail
    }
    else
    {
        return rb->capacity + rb->head - rb->tail; // head ha envuelto alrededor de tail
    }
}

/**
 * función: Indica si el buffer está vacío.
 *
 * rb  Puntero al ring buffer.
 * true  Si no hay datos almacenados.
 * false Si contiene al menos un dato.
 */
bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    return rb->head == rb->tail;
}

/**
 * función: Indica si el buffer está lleno.
 *
 * rb  Puntero al ring buffer.
 * true  Si no se pueden escribir más datos.
 * false Si aún hay espacio disponible.
 */
bool ring_buffer_is_full(ring_buffer_t *rb)
{
    // Está lleno si avanzar head haría que coincida con tail
    return ((rb->head + 1) % rb->capacity) == rb->tail;
}

/**
 * función: Vacía el buffer circular.
 *
 * rb  Puntero al ring buffer.
 *
 * Reinicia los punteros de lectura y escritura.
 * Los datos almacenados se sobrescriben en las siguientes escrituras.
 */
void ring_buffer_flush(ring_buffer_t *rb)
{
    rb->head = 0;        // Reinicia el puntero de escritura
    rb->tail = 0;        // Reinicia el puntero de lectura
    rb->is_full = false; // Marca el buffer como no lleno
}
