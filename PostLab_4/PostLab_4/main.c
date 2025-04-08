/*
 * LecturaADC_Displays.c - Versión optimizada con referencia VCC
 *
 * Created: 03/04/2025
 * Author: Jesus Valenzuela (con optimizaciones)
 * Description: Sistema con funcionalidades separadas:
 *              - Potenciómetro en A6 controla displays de 7 segmentos en formato hexadecimal
 *              - Botones controlan contador binario mostrado en LEDs
 *              - Multiplexado mediante PORTC para controlar los displays y LEDs
 *              - Antirebote implementado por hardware
 *              - Utilizando VCC (5V) como referencia del ADC
 *              - Optimizado para estabilidad y menor uso de recursos
 *              - LED de alarma en PORTB5 que se enciende cuando el valor del ADC > contador LEDs
 */

/************************************/
//ENCABEZADO (Libraries)
/************************************/
#define F_CPU 16000000UL  // 16 MHz - Define la frecuencia del CPU para cálculos internos

#include <avr/io.h>         // Librería para acceso a registros del microcontrolador
#include <avr/interrupt.h>  // Librería para manejo de interrupciones

// Definición de pines - Mejora la legibilidad y mantenimiento del código
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
#define ALARM_LED PB5     // LED de alarma conectado a PORTB5

// Variables globales
volatile uint8_t contador_leds = 0;     // Contador para los LEDs (controlado por botones)
volatile uint8_t contador_displays = 0; // Valor para los displays (controlado por ADC)
volatile uint16_t adc_value = 0;        // Valor bruto leído del ADC
volatile uint8_t adc_ready = 0;         // Bandera para indicar cuando hay una nueva conversión ADC completa

// Tabla de conversión para displays de 7 segmentos (cátodo común) en formato hexadecimal
// Segmentos: ABCDEFG (activo en ALTO para cátodo común)
// Formato hexadecimal es más compacto y fácil de leer que el binario
const uint8_t display_7seg[16] = {
    0x3F, // 0 - Segmentos ABCDEF encendidos (0b00111111)
    0x06, // 1 - Segmentos BC encendidos (0b00000110)
    0x5B, // 2 - Segmentos ABDEG encendidos (0b01011011)
    0x4F, // 3 - Segmentos ABCDG encendidos (0b01001111)
    0x66, // 4 - Segmentos BCFG encendidos (0b01100110)
    0x6D, // 5 - Segmentos ACDFG encendidos (0b01101101)
    0x7D, // 6 - Segmentos ACDEFG encendidos (0b01111101)
    0x07, // 7 - Segmentos ABC encendidos (0b00000111)
    0x7F, // 8 - Todos los segmentos encendidos (0b01111111)
    0x6F, // 9 - Segmentos ABCDFG encendidos (0b01101111)
    0x77, // A - Segmentos ABCEFG encendidos (0b01110111)
    0x7C, // b - Segmentos CDEFG encendidos (0b01111100)
    0x39, // C - Segmentos ADEF encendidos (0b00111001)
    0x5E, // d - Segmentos BCDEG encendidos (0b01011110)
    0x79, // E - Segmentos ADEFG encendidos (0b01111001)
    0x71  // F - Segmentos AEFG encendidos (0b01110001)
};

/*********************************/
// PROTOTIPO DE FUNCIONES
/*********************************/
void deshabilitar_uart(void);         // Deshabilita el UART para usar RX/TX como I/O
void inicializar_pines(void);         // Configura los pines del microcontrolador
void inicializar_adc(void);           // Configura el conversor analógico-digital
void inicializar_timer0(void);        // Configura el Timer0 para el multiplexado
void iniciar_conversion_adc(void);    // Inicia una nueva conversión ADC
void actualizar_led_alarma(void);     // Actualiza el estado del LED de alarma

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
    
    // Iniciar primera conversión ADC - El resto será automático mediante interrupciones
    iniciar_conversion_adc();
    
    // Habilitar interrupciones globales - Activa el sistema de interrupciones
    sei();
    
    // Bucle principal - Optimizado, ya no usa delays
    while (1) {
        // Comprobar si hay una nueva lectura de ADC disponible usando la bandera
        if (adc_ready) {
            // Convertir valor ADC (10 bits) a valor de 8 bits para los displays
            // Divide por 4 usando desplazamiento de bits (más eficiente que división)
            contador_displays = (uint8_t)(adc_value >> 2);
            adc_ready = 0;  // Reiniciar la bandera
            
            // Actualizar el LED de alarma basado en la comparación entre contador_displays y contador_leds
            actualizar_led_alarma();
            
            // Iniciar siguiente conversión ADC
            iniciar_conversion_adc();
        }
        
        // Aquí podrían realizarse otras tareas mientras esperamos la conversión ADC
        // Este bucle ya no está bloqueado por delay_ms() o por espera activa
    }
    
    return 0;  // Nunca se llega a este punto en aplicaciones embebidas
}

/*************************************/
// NON-INTERRUPT SUBROUTINES 
/*************************************/
// Función para deshabilitar UART - Necesario para usar PD0/PD1 como I/O normales
void deshabilitar_uart(void) {
    // Deshabilitar USART
    UCSR0B = 0x00;  // Deshabilitar transmisor y receptor
    
    // Configurar pines RX y TX como I/O digitales normales
    LED_DDR |= (1 << RX_PIN) | (1 << TX_PIN);
}

// Función para inicializar los pines
void inicializar_pines(void) {
    // Deshabilitar UART primero para poder usar RX/TX
    deshabilitar_uart();
    
    // Configurar los pines de los LEDs y displays como salidas (PORTD0-PORTD7)
    LED_DDR = 0xFF;  // 0xFF = todos los bits a 1 = todos como salidas
    LED_PORT = 0x00; // Inicialmente todos los LEDs apagados
    
    // Configurar pines de multiplexado como salidas
    MUX_DDR |= (1 << MUX_LEDS) | (1 << MUX_DISP1) | (1 << MUX_DISP2);
    // Inicialmente todos desactivados (nivel bajo)
    MUX_PORT &= ~((1 << MUX_LEDS) | (1 << MUX_DISP1) | (1 << MUX_DISP2)); 
    
    // Configurar los pines de los botones como entradas con pull-up
    DDRB &= ~(1 << BTN_INCREMENT) & ~(1 << BTN_DECREMENT);  // Configurar como entradas
    PORTB |= (1 << BTN_INCREMENT) | (1 << BTN_DECREMENT);   // Activar resistencias pull-up
    
    // Configurar pin change interrupt para PORTB
    PCICR |= (1 << PCIE0);   // Habilitar interrupciones para PORTB
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);  // Habilitar interrupciones para PB0 y PB1
    
    // Configurar el pin del LED de alarma como salida
    DDRB |= (1 << ALARM_LED);     // Configura PORTB5 como salida
    PORTB &= ~(1 << ALARM_LED);   // Inicialmente apagado
}

// Función para inicializar el ADC
void inicializar_adc(void) {
    // Configurar referencia a AVCC (VCC) con capacitor en AREF
    ADMUX = (1 << REFS0);  // AVCC como referencia, ajuste a la derecha (ADLAR=0)
    
    // Seleccionar canal ADC6 manteniendo la configuración de referencia
    ADMUX = (ADMUX & 0xF0) | (ADC_CHANNEL & 0x0F);
    
    // Habilitar ADC con interrupciones y prescaler 128
    // ADEN: ADC Enable - Activa el ADC
    // ADIE: ADC Interrupt Enable - Habilita las interrupciones del ADC
    // ADPS2-0: ADC Prescaler Select - Divisor de 128 para frecuencia ADC entre 50-200kHz
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Función para inicializar Timer0 para multiplexado
void inicializar_timer0(void) {
    // Configurar Timer0 en modo CTC (Clear Timer on Compare Match)
    TCCR0A = (1 << WGM01);
    
    // Prescaler 64 - Divide la frecuencia del reloj por 64
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // Comparación para aproximadamente 1ms
    // Cálculo: (16MHz / 64) / 1000Hz - 1 = 249
    OCR0A = 249;
    
    // Habilitar interrupción por comparación - Se ejecutará cada 1ms
    TIMSK0 = (1 << OCIE0A);
}

// Función para iniciar una conversión ADC - Simplificada vs. versión original
void iniciar_conversion_adc(void) {
    // Iniciar conversión (ADC Start Conversion)
    ADCSRA |= (1 << ADSC);
    // No esperamos a que termine - La interrupción ADC_vect se ocupará de eso
}

// Función para actualizar el estado del LED de alarma
void actualizar_led_alarma(void) {
    // Compara el valor del ADC (contador_displays) con el contador binario (contador_leds)
    if (contador_displays > contador_leds) {
        // Si es mayor, enciende el LED de alarma
        PORTB |= (1 << ALARM_LED);
    } else {
        // Si es menor o igual, apaga el LED de alarma
        PORTB &= ~(1 << ALARM_LED);
    }
}

/******************************************/
// RUTINAS DE INTERRUPCIÓN
/******************************************/
// Rutina de servicio de interrupción para cambios en PORTB (botones)
ISR(PCINT0_vect) {
    // Como el antirebote es por hardware, podemos procesar directamente
    
    // Verificar si el botón de incremento ha sido presionado (activo en bajo)
    if (!(PINB & (1 << BTN_INCREMENT))) {
        contador_leds++;  // Solo incrementa el contador de LEDs
        actualizar_led_alarma();  // Actualizar el LED de alarma al cambiar contador_leds
    }
    
    // Verificar si el botón de decremento ha sido presionado (activo en bajo)
    if (!(PINB & (1 << BTN_DECREMENT))) {
        contador_leds--;  // Solo decrementa el contador de LEDs
        actualizar_led_alarma();  // Actualizar el LED de alarma al cambiar contador_leds
    }
}

// Rutina de interrupción para ADC completo - Maneja la conversión finalizada
ISR(ADC_vect) {
    // Guardar el resultado ADC
    adc_value = ADC;  // ADC es el registro que contiene el valor de conversión
    
    // OPTIMIZACIÓN: Filtro para estabilizar la lectura (promedio móvil simple)
    // Reduce el parpadeo en los displays causado por fluctuaciones
    static uint16_t adc_values[4] = {0, 0, 0, 0};  // Almacena las últimas 4 lecturas
    static uint8_t adc_index = 0;                  // Índice para actualizar el arreglo circular
    
    // Actualizar el arreglo circular con el nuevo valor
    adc_values[adc_index] = adc_value;
    // Incrementar el índice y volver a 0 al llegar a 4 (modulo 4)
    // Optimizado usando operación AND bit a bit (más rápido que división)
    adc_index = (adc_index + 1) & 0x03;  // Equivalente a (adc_index + 1) % 4
    
    // Calcular promedio dividiendo la suma por 4
    // Usamos shift right (>>) que es más eficiente que la división
    adc_value = (adc_values[0] + adc_values[1] + adc_values[2] + adc_values[3]) >> 2;
    
    // Marcar que hay un nuevo valor disponible para el bucle principal
    adc_ready = 1;
}

// Rutina de interrupción para Timer0 (multiplexado de displays)
// Se ejecuta aproximadamente cada 1ms para actualizar la pantalla
ISR(TIMER0_COMPA_vect) {
    static uint8_t mux_state = 0;  // Estado de multiplexado (0-2)
    
    // Desactivar todos los dispositivos antes de actualizar
    MUX_PORT &= ~((1 << MUX_LEDS) | (1 << MUX_DISP1) | (1 << MUX_DISP2));
    
    // Multiplexado rotativo entre LEDs y displays
    switch (mux_state) {
        case 0: // Mostrar LEDs controlados por botones
            LED_PORT = contador_leds;  // Muestra el valor binario en LEDs
            MUX_PORT |= (1 << MUX_LEDS);  // Activa el multiplexor para LEDs
            break;
            
        case 1: // Mostrar dígito de unidades hexadecimal (Display 1) - controlado por ADC
            // Obtiene los 4 bits menos significativos y busca el patrón correspondiente
            LED_PORT = display_7seg[contador_displays & 0x0F];
            MUX_PORT |= (1 << MUX_DISP1);  // Activa el display 1
            break;
            
        case 2: // Mostrar dígito de decenas hexadecimal (Display 2) - controlado por ADC
            // Obtiene los 4 bits más significativos y busca el patrón correspondiente
            LED_PORT = display_7seg[(contador_displays >> 4) & 0x0F];
            MUX_PORT |= (1 << MUX_DISP2);  // Activa el display 2
            break;
    }
    
    // Avanzar al siguiente estado (0->1->2->0...)
    mux_state = (mux_state + 1) % 3;  // Cicla entre 0, 1 y 2
}

