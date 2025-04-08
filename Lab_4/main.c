/*
 * LecturaADC_Displays.c - Versión con referencia VCC
 *
 * Created: 01/04/2025
 * Author: Jesus Valenzuela (con mejoras sugeridas)
 * Description: Sistema con funcionalidades separadas:
 *              - Potenciómetro en A6 controla displays de 7 segmentos en formato hexadecimal
 *              - Botones controlan contador binario mostrado en LEDs
 *              - Multiplexado mediante PORTC para controlar los displays y LEDs
 *              - Antirebote implementado por hardware
 *              - Utilizando VCC (5V) como referencia del ADC
 */
/************************************/
//ENCABEZADO (Libraries)
/************************************/
#define F_CPU 16000000UL  // 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Definición de pines
#define BTN_INCREMENT PB0 // Botón para incrementar en PORTB0
#define BTN_DECREMENT PB1 // Botón para decrementar en PORTB1
#define LED_PORT PORTD    // Puerto D para los LEDs y displays
#define LED_DDR DDRD      // Registro de dirección de datos para los LEDs
#define MUX_PORT PORTC    // Puerto C para el multiplexado
#define MUX_DDR DDRC      // Registro de dirección para multiplexado
#define RX_PIN PD0        // Pin RX (PORTD0)
#define TX_PIN PD1        // Pin TX (PORTD1)
#define MUX_LEDS PC1      // Pin para seleccionar LEDs
#define MUX_DISP1 PC2     // Pin para seleccionar Display 1
#define MUX_DISP2 PC3     // Pin para seleccionar Display 2
#define ADC_CHANNEL 6     // Canal ADC (ADC6) para el potenciómetro

// Variables globales
volatile uint8_t contador_leds = 0;     // Contador para los LEDs (controlado por botones)
volatile uint8_t contador_displays = 0; // Valor para los displays (controlado por ADC)
volatile uint16_t adc_value = 0;

// Tabla de conversión para displays de 7 segmentos (común cátodo)
// Segmentos: ABCDEFG (activo en ALTO para cátodo común)
const uint8_t display_7seg[16] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01110111, // A
    0b01111100, // b
    0b00111001, // C
    0b01011110, // d
    0b01111001, // E
    0b01110001  // F
};

/*********************************/
// PROTOTIPO DE FUNCIONES
/*********************************/
void deshabilitar_uart(void);
void inicializar_pines(void);
void inicializar_adc(void);
void inicializar_timer0(void);
uint16_t leer_adc(void);

/*********************************/
// MAIN FUNCTION
/*********************************/
int main(void) {
    // Inicializar pines
    inicializar_pines();
    
    // Inicializar ADC
    inicializar_adc();
    
    // Inicializar Timer0 para multiplexado
    inicializar_timer0();
    
    // Habilitar interrupciones globales
    sei();
    
    // Bucle principal
    while (1) {
        // Leer valor del ADC
        adc_value = leer_adc();
        
        // Convertir valor ADC (10 bits) a valor de 8 bits para los displays
        contador_displays = (uint8_t)(adc_value >> 2);
        
        // Pequeña pausa para no leer el ADC constantemente
        _delay_ms(50);
    }
    
    return 0;
}

/*************************************/
// NON-INTERRUPT SUBROUTINES 
/*************************************/
// Función para deshabilitar UART
void deshabilitar_uart(void) {
    // Deshabilitar USART
    UCSR0B = 0x00;  // Deshabilitar transmisor y receptor
    
    // Configurar pines RX y TX como I/O digitales normales
    LED_DDR |= (1 << RX_PIN) | (1 << TX_PIN);
}

// Función para inicializar los pines
void inicializar_pines(void) {
    // Deshabilitar UART primero
    deshabilitar_uart();
    
    // Configurar los pines de los LEDs y displays como salidas (PORTD0-PORTD7)
    LED_DDR = 0xFF;
    LED_PORT = 0x00; // Inicialmente todos los LEDs apagados
    
    // Configurar pines de multiplexado como salidas
    MUX_DDR |= (1 << MUX_LEDS) | (1 << MUX_DISP1) | (1 << MUX_DISP2);
    MUX_PORT &= ~((1 << MUX_LEDS) | (1 << MUX_DISP1) | (1 << MUX_DISP2)); // Inicialmente todos desactivados
    
    // Configurar los pines de los botones como entradas con pull-up
    DDRB &= ~(1 << BTN_INCREMENT) & ~(1 << BTN_DECREMENT);
    PORTB |= (1 << BTN_INCREMENT) | (1 << BTN_DECREMENT);
    
    // Configurar pin change interrupt para PORTB
    PCICR |= (1 << PCIE0);   // Habilitar interrupciones para PORTB
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);  // Habilitar interrupciones para PB0 y PB1
}

// Función para inicializar el ADC
void inicializar_adc(void) {
	// Configurar referencia a AVCC (VCC) con capacitor en AREF
	// Para ATmega328PB, REFS1=0 y REFS0=1 selecciona AVCC como referencia
	ADMUX = (1 << REFS0);  // AVCC como referencia, ajuste a la derecha (ADLAR=0)
	
	// Seleccionar canal ADC6 manteniendo la configuración de referencia
	ADMUX = (ADMUX & 0xF0) | (ADC_CHANNEL & 0x0F);
	
	// Habilitar ADC, prescaler 128 para frecuencia ADC entre 50-200kHz
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Función para inicializar Timer0 para multiplexado
void inicializar_timer0(void) {
    // Configurar Timer0 en modo CTC
    TCCR0A = (1 << WGM01);
    
    // Prescaler 64
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // Comparación para aproximadamente 1ms
    OCR0A = 249;  // (16MHz / 64) / 1000Hz - 1 = 249
    
    // Habilitar interrupción por comparación
    TIMSK0 = (1 << OCIE0A);
}

// Función para leer el ADC
uint16_t leer_adc(void) {
    // Iniciar conversión
    ADCSRA |= (1 << ADSC);
    
    // Esperar a que termine la conversión
    while (ADCSRA & (1 << ADSC));
    
    // Retornar resultado
    return ADC;
}

/******************************************/
// RUTINAS DE INTERRUPCIÓN
/******************************************/
// Rutina de servicio de interrupción para cambios en PORTB
ISR(PCINT0_vect) {
    // Como el antirebote es por hardware, podemos procesar directamente
    // Verificar qué botón ha sido presionado (solo afecta a contador_leds)
    if (!(PINB & (1 << BTN_INCREMENT))) {
        contador_leds++;  // Solo incrementa el contador de LEDs
    }
    
    if (!(PINB & (1 << BTN_DECREMENT))) {
        contador_leds--;  // Solo decrementa el contador de LEDs
    }
}

// Rutina de interrupción para Timer0 (multiplexado de displays)
ISR(TIMER0_COMPA_vect) {
    static uint8_t mux_state = 0;
    
    // Desactivar todos los dispositivos
    MUX_PORT &= ~((1 << MUX_LEDS) | (1 << MUX_DISP1) | (1 << MUX_DISP2));
    
    // Multiplexado rotativo entre LEDs y displays
    switch (mux_state) {
        case 0: // Mostrar LEDs controlados por botones
            LED_PORT = contador_leds;
            MUX_PORT |= (1 << MUX_LEDS);
            break;
            
        case 1: // Mostrar dígito de unidades hexadecimal (Display 1) - controlado por ADC
            LED_PORT = display_7seg[contador_displays & 0x0F];
            MUX_PORT |= (1 << MUX_DISP1);
            break;
            
        case 2: // Mostrar dígito de decenas hexadecimal (Display 2) - controlado por ADC
            LED_PORT = display_7seg[(contador_displays >> 4) & 0x0F];
            MUX_PORT |= (1 << MUX_DISP2);
            break;
    }
    
    // Avanzar al siguiente estado
    mux_state = (mux_state + 1) % 3;
}

