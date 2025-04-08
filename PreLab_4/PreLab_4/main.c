/*
 * ContadorBinario.c
 *
 * Created: 01/04/2025
 * Author: Jesus Valenzuela
 * Description: Contador binario de 8 bits controlado por dos pulsadores.
 *              Utiliza interrupciones PCINT para detectar pulsaciones y
 *              muestra el resultado en 8 LEDs conectados al PORTD.
 */

/************************************/
//ENCABEZADO (Libraries)
/************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

// Definición de pines
#define BTN_INCREMENT PB0 // Botón para incrementar en PORTB0
#define BTN_DECREMENT PB1 // Botón para decrementar en PORTB1
#define LED_PORT PORTD    // Puerto D para los LEDs
#define LED_DDR DDRD      // Registro de dirección de datos para los LEDs
#define RX_PIN PD0        // Pin RX (PORTD0)
#define TX_PIN PD1        // Pin TX (PORTD1)

// Variables globales
volatile uint8_t contador = 0;

/*********************************/
// PROTOTIPO DE FUNCIONES
/*********************************/
void deshabilitar_uart(void);
void inicializar_pines(void);
void actualizar_leds(void);

/*********************************/
// MAIN FUNCTION
/*********************************/
int main(void) {
    // Inicializar pines
    inicializar_pines();
    
    // Actualizar los LEDs con el valor inicial
    actualizar_leds();
    
    // Habilitar interrupciones globales
    sei();
    
    // Bucle principal
    while (1) {
        // El programa solo responde a las interrupciones
        // No es necesario hacer nada aquí
    }
    
    return 0;
}

/*************************************/
// NON- INTERRUPT SUBROUTINES 
/*************************************/
// Función para deshabilitar UART
void deshabilitar_uart(void) {
    // Deshabilitar USART
    UCSR0B = 0x00;  // Deshabilitar transmisor y receptor
    
    // Configurar pines RX y TX como I/O digitales normales
    // RX (PD0) y TX (PD1) ahora pueden usarse como salidas normales
    LED_DDR |= (1 << RX_PIN) | (1 << TX_PIN);
}

// Función para inicializar los pines
void inicializar_pines(void) {
    // Deshabilitar UART primero
    deshabilitar_uart();
    
    // Configurar los pines de los LEDs como salidas (PORTD0-PORTD7)
    LED_DDR = 0xFF;
    LED_PORT = 0x00; // Inicialmente todos los LEDs apagados
    
    // Configurar los pines de los botones como entradas con pull-up
    DDRB &= ~(1 << BTN_INCREMENT) & ~(1 << BTN_DECREMENT);
    PORTB |= (1 << BTN_INCREMENT) | (1 << BTN_DECREMENT);
    
    // Configurar pin change interrupt para PORTB
    PCICR |= (1 << PCIE0);   // Habilitar interrupciones para PORTB
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);  // Habilitar interrupciones para PB0 y PB1
}

// Función para actualizar los LEDs
void actualizar_leds(void) {
    LED_PORT = contador;
}

/******************************************/
// RUTINAS DE INTERRUPCIÓN
/******************************************/
// Rutina de servicio de interrupción para cambios en PORTB
ISR(PCINT0_vect) {
    // Verificar qué botón ha sido presionado
    if (!(PINB & (1 << BTN_INCREMENT))) {
        contador++;
        actualizar_leds();
    }
    
    if (!(PINB & (1 << BTN_DECREMENT))) {
        contador--;
        actualizar_leds();
    }
}

