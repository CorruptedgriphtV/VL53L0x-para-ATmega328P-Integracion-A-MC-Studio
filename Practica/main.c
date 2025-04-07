#ifndef F_CPU
    #define F_CPU 16000000UL // Frecuencia del cristal (solo si no está definida)
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h> // Para atof()
#include <string.h> // Para strncmp()
#include <math.h>   // Para fabs()

// --- Definiciones del Microcontrolador y Comunicación ---
#define BAUD 115200      // Velocidad de comunicación UART en baudios
#define UBRR_VALUE ((F_CPU)/(16UL*BAUD)-1) // Valor para el registro UBRR0 para la velocidad BAUD

// --- Definiciones del Sistema Físico ---
#define PULSOS_POR_REVOLUCION_ENCODER 300 // Número de pulsos que genera el encoder por cada revolución completa
#define GRADOS_POR_PULSO (360.0 / PULSOS_POR_REVOLUCION_ENCODER) // Grados de rotación por cada pulso del encoder

// --- Definiciones de Pines ---
#define PWM_PIN PB1   // Pin de salida PWM para controlar la velocidad del motor (OC1A)
#define AIN1_PIN PD7  // Pin de control 1 del puente H (dirección)
#define AIN2_PIN PD6  // Pin de control 2 del puente H (dirección)
#define ENCODER_A_PIN PD2 // Pin para la señal A del encoder (INT0)
#define ENCODER_B_PIN PD3 // Pin para la señal B del encoder
#define HALL_SENSOR_PIN PC5 // Pin para el sensor de efecto Hall (PCINT13)

// --- Variables Globales Volátiles (Modificadas por ISRs) ---
volatile int32_t contador_pulsos = 0; // Contador de pulsos del encoder (puede ser negativo)
volatile uint8_t hall_activado = 0;   // Bandera que indica si el sensor Hall se activó (0 = no, 1 = sí)
volatile uint8_t calibrado = 0;       // Bandera que indica si el sistema se ha calibrado (0 = no, 1 = sí)

// --- Parámetros de Control ---
volatile float ANGULO_DESEADO = 90.0; // Ángulo objetivo inicial (puede ser modificado por UART)
float kp = 0.2; // Ganancia Proporcional inicial (puede ser modificada por UART)
float kd = 0.9; // Ganancia Derivativa inicial (puede ser modificada por UART)

// --- Constantes de Control ---
#define UMBRAL_ESTABILIDAD 1.0 // Margen de error aceptable para considerar que se alcanzó el ángulo deseado

//////////////////////////////////////////////////////////////////////////
// Funciones de Comunicación UART
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa el hardware UART del microcontrolador.
 */
void uart_inicializar(void) {
    // Configura la velocidad (baud rate)
    UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
    UBRR0L = (unsigned char)UBRR_VALUE;
    // Habilita la recepción (RXEN0) y transmisión (TXEN0)
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Configura el formato de trama: 8 bits de datos, 1 bit de parada (por defecto)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/**
 * @brief Transmite un solo carácter por UART.
 * @param dato El carácter a transmitir.
 */
void uart_transmitir(unsigned char dato) {
    // Espera hasta que el buffer de transmisión esté vacío (UDRE0 = 1)
    while (!(UCSR0A & (1 << UDRE0)));
    // Carga el dato en el registro de transmisión UDR0
    UDR0 = dato;
}

/**
 * @brief Transmite una cadena de caracteres (string) por UART.
 * @param str Puntero a la cadena de caracteres terminada en null.
 */
void uart_imprimir(const char* str) {
    // Itera sobre la cadena hasta encontrar el carácter nulo
    while(*str) {
        uart_transmitir(*str++); // Transmite cada carácter
    }
}

/**
 * @brief Lee una línea de texto desde UART hasta encontrar un retorno de carro o nueva línea.
 *        Almacena la línea en el buffer proporcionado. Es bloqueante.
 * @param buffer Puntero al buffer donde se almacenará la línea leída.
 * @param max_longitud Tamaño máximo del buffer (incluyendo el carácter nulo).
 */
void uart_leer_linea(char *buffer, uint8_t max_longitud) {
    uint8_t i = 0;
    char c;
    while (i < (max_longitud - 1)) {
        // Espera hasta que se reciba un carácter (RXC0 = 1)
        while (!(UCSR0A & (1 << RXC0)));
        c = UDR0; // Lee el carácter recibido
        // Si es retorno de carro o nueva línea, termina la lectura
        if(c == '\r' || c == '\n')
            break;
        buffer[i++] = c; // Almacena el carácter en el buffer
    }
    buffer[i] = '\0'; // Asegura que la cadena esté terminada en null
}

/**
 * @brief Descarta cualquier dato pendiente en el buffer de recepción UART.
 */
void limpiar_buffer_uart(void) {
    while (UCSR0A & (1 << RXC0)) {
        (void)UDR0; // Leer y descartar el dato
    }
}

//////////////////////////////////////////////////////////////////////////
// Funciones de PWM y Control del Puente H (Motor)
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Configura el Timer1 para generar una señal PWM en el pin PWM_PIN (OC1A).
 */
void configurar_pwm(void) {
    // Configura el pin PWM como salida
    DDRB |= (1 << PWM_PIN);
    // Configura el Timer1 en modo PWM Rápido de 8 bits (WGM10=1, WGM12=1)
    // TOP = 0xFF (255)
    TCCR1A |= (1 << WGM10);
	TCCR1B |= (1 << WGM12); // Corrección: WGM12 está en TCCR1B
    // Configura la salida OC1A para modo no invertido (COM1A1=1)
    // El PWM se pone en alto en la comparación y en bajo en BOTTOM.
    TCCR1A |= (1 << COM1A1);
    // Configura el preescalador del Timer1 a 1 (CS10=1) para iniciar el temporizador
    // Frecuencia PWM = F_CPU / (Preescalador * (TOP + 1)) = 16MHz / (1 * 256) = 62.5 kHz
    TCCR1B |= (1 << CS10);
    // Establece un ciclo de trabajo inicial (Duty Cycle)
    // OCR1A controla el ancho del pulso. Valor entre 0 y 255.
    OCR1A = 50; // Ejemplo: ~20% de ciclo de trabajo (50/255)
}

/**
 * @brief Configura los pines de control del puente H como salidas.
 */
void configurar_puente_h(void) {
    DDRD |= (1 << AIN1_PIN) | (1 << AIN2_PIN); // Configura los pines como salida
}

/**
 * @brief Configura el puente H para mover el motor en un sentido (ej. "adelante").
 */
void mover_motor_adelante(void) {
    PORTD |= (1 << AIN2_PIN);  // AIN2 = ALTO
    PORTD &= ~(1 << AIN1_PIN); // AIN1 = BAJO
}

/**
 * @brief Configura el puente H para mover el motor en el sentido opuesto (ej. "atrás").
 */
void mover_motor_atras(void) {
    PORTD |= (1 << AIN1_PIN);  // AIN1 = ALTO
    PORTD &= ~(1 << AIN2_PIN); // AIN2 = BAJO
}

/**
 * @brief Configura el puente H para detener el motor (freno bajo).
 */
void detener_motor(void) {
    PORTD &= ~(1 << AIN1_PIN); // AIN1 = BAJO
    PORTD &= ~(1 << AIN2_PIN); // AIN2 = BAJO
}

//////////////////////////////////////////////////////////////////////////
// ISRs (Rutinas de Servicio de Interrupción)
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ISR para la interrupción externa INT0 (conectada a ENCODER_A_PIN).
 *        Se activa en cada flanco de subida de la señal A del encoder.
 *        Lee la señal B para determinar la dirección y actualiza contador_pulsos.
 */
ISR(INT0_vect) {
    // Lee el estado del pin B del encoder (ENCODER_B_PIN)
    if (PIND & (1 << ENCODER_B_PIN)) { // Si la señal B está en ALTO
        contador_pulsos--; // Decrementa el contador (ej. rotación antihoraria)
    } else { // Si la señal B está en BAJO
        contador_pulsos++; // Incrementa el contador (ej. rotación horaria)
    }
}

/**
 * @brief ISR para la interrupción por cambio de pin PCINT1 (grupo PCINT8-PCINT14).
 *        Se activa cuando hay un cambio en cualquier pin habilitado del grupo.
 *        Verifica específicamente el estado del sensor Hall (HALL_SENSOR_PIN).
 */
ISR(PCINT1_vect) {
    // Verifica si el pin del sensor Hall (PC5) está en BAJO (activo)
    if (!(PINC & (1 << HALL_SENSOR_PIN))) {
        hall_activado = 1; // Establece la bandera indicando que el Hall se activó
    }
    // Nota: Esta ISR se activará en CUALQUIER cambio de los pines PCINT8-14.
    // Si otros pines de este grupo se usan y cambian, esta ISR se ejecutará.
    // La lógica actual solo reacciona al estado de PC5 cuando la ISR se dispara.
}

//////////////////////////////////////////////////////////////////////////
// Función Principal (MAIN)
//////////////////////////////////////////////////////////////////////////
int main(void) {
    // Buffers para comunicación UART y conversión de números
    char buffer_uart[80];    // Buffer general para formatear mensajes UART
    char buffer_entrada[32]; // Buffer para leer comandos/datos de UART
    char angulo_str[10];     // Buffer para convertir ángulo actual a cadena
    char error_str[10];      // Buffer para convertir error a cadena

    // --- Inicializaciones de Periféricos ---
    uart_inicializar();      // Configura la comunicación UART
    configurar_pwm();        // Configura el PWM para el control del motor
    configurar_puente_h();   // Configura los pines de control del puente H

    // --- Configuración de Pines de Entrada (Encoder y Sensor Hall) ---
    // Configuración del pin INT0 (PD2) para el encoder (canal A)
    DDRD &= ~(1 << ENCODER_A_PIN); // PD2 como entrada
    PORTD |= (1 << ENCODER_A_PIN); // Habilita pull-up interno en PD2

    // Configuración del pin PD3 para el encoder (canal B)
    DDRD &= ~(1 << ENCODER_B_PIN); // PD3 como entrada
    PORTD |= (1 << ENCODER_B_PIN); // Habilita pull-up interno en PD3

    // Configuración del pin PC5 para el sensor Hall
    DDRC &= ~(1 << HALL_SENSOR_PIN); // PC5 como entrada
    PORTC |= (1 << HALL_SENSOR_PIN); // Habilita pull-up interno en PC5

    // --- Configuración de Interrupciones ---
    // Configuración de la interrupción externa INT0 (Encoder A)
    EICRA |= (1 << ISC01) | (1 << ISC00); // INT0 se activa en flanco de subida
    EIMSK |= (1 << INT0);                 // Habilita la interrupción INT0

    // Configuración de la interrupción por cambio de pin para PC5 (Sensor Hall)
    PCMSK1 |= (1 << PCINT13); // Habilita la interrupción para PCINT13 (correspondiente a PC5)
    PCICR |= (1 << PCIE1);    // Habilita las interrupciones del grupo PCINT8-PCINT14 (Puerto C)

    sei(); // Habilita las interrupciones globales

    // --- Calibración Inicial ---
    // Mueve el motor hasta que se active el sensor Hall para encontrar la posición cero.
    uart_imprimir("Iniciando calibracion...\r\n");
    mover_motor_adelante(); // Comienza a mover el motor
    OCR1A = 60; // Establece un ciclo de trabajo moderado para la calibración
    while (!hall_activado) { // Espera hasta que la ISR del Hall active la bandera
        _delay_ms(10); // Pequeña pausa para no saturar el CPU
    }
    detener_motor(); // Detiene el motor una vez encontrado el punto de referencia
    _delay_ms(100); // Pausa para asegurar que el motor se detenga completamente
    uart_imprimir("Calibracion completada. Sensor Hall detectado.\r\n");

    contador_pulsos = 0; // Reinicia el contador de pulsos en la posición de referencia (cero)
    calibrado = 1;       // Marca que la calibración se ha completado
    hall_activado = 0;   // Reinicia la bandera del sensor Hall para futuras detecciones

    // --- Bucle Principal de Control ---
    float angulo_actual, error; // Variables para el ángulo y el error
    float error_anterior = 0.0; // Variable para almacenar el error anterior para el cálculo derivativo
    float derivada_error;       // Variable para la derivada del error
    float senal_control;        // Salida del controlador PD
    int pwm_valor;              // Valor a escribir en OCR1A (0-255)
    uint8_t hall_anterior = 1;  // Estado anterior del sensor Hall (1 = inactivo, 0 = activo)

    uart_imprimir("Sistema listo. Esperando comandos (ej: KP=0.5, KD=0.8, AN=180.0)\r\n");

    while (1) { // Bucle infinito

        // --- Lectura de Comandos UART (No bloqueante) ---
        if (UCSR0A & (1 << RXC0)) { // Verifica si hay datos recibidos en UART
            uart_leer_linea(buffer_entrada, sizeof(buffer_entrada)); // Lee la línea de comando

            // Procesa el comando recibido
            if (strncmp(buffer_entrada, "KP=", 3) == 0) { // Comando para Kp
                kp = atof(buffer_entrada + 3); // Convierte la parte numérica y actualiza Kp
                sprintf(buffer_uart, "Nuevo Kp: %f\r\n", kp);
                uart_imprimir(buffer_uart);
            } else if (strncmp(buffer_entrada, "KD=", 3) == 0) { // Comando para Kd
                kd = atof(buffer_entrada + 3); // Convierte y actualiza Kd
                sprintf(buffer_uart, "Nuevo Kd: %f\r\n", kd);
                uart_imprimir(buffer_uart);
            } else if (strncmp(buffer_entrada, "AN=", 3) == 0) { // Comando para Ángulo Deseado
                ANGULO_DESEADO = atof(buffer_entrada + 3); // Convierte y actualiza el ángulo deseado
                sprintf(buffer_uart, "Nuevo Angulo Deseado: %f\r\n", ANGULO_DESEADO);
                uart_imprimir(buffer_uart);
            } else if (strlen(buffer_entrada) > 0) { // Si se recibió algo pero no es un comando conocido
                uart_imprimir("Comando no reconocido: ");
                uart_imprimir(buffer_entrada);
                uart_imprimir("\r\n");
            }
             // Si no se recibió nada o solo un Enter, no hace nada.
        }

        // --- Detección de paso por cero (Sensor Hall) ---
        // Lee el estado actual del sensor Hall (activo bajo)
        uint8_t hall_actual = (PINC & (1 << HALL_SENSOR_PIN)) ? 1 : 0; // 1 si está inactivo (alto), 0 si está activo (bajo)
        // Si el estado anterior era inactivo (1) y el actual es activo (0), significa que pasó por el punto cero
        if (hall_anterior == 1 && hall_actual == 0) {
            contador_pulsos = 0; // Reinicia el contador de pulsos
            // uart_imprimir("Paso por cero detectado (Hall).\r\n"); // Mensaje opcional de depuración
        }
        hall_anterior = hall_actual; // Actualiza el estado anterior del Hall para la próxima detección

        // --- Cálculo del Error y Control PD ---
        angulo_actual = contador_pulsos * GRADOS_POR_PULSO; // Calcula el ángulo actual
        error = ANGULO_DESEADO - angulo_actual; // Calcula el error Proporcional

        // --- Cálculo del Término Derivativo ---
        derivada_error = error - error_anterior; // Calcula la diferencia de error respecto a la iteración anterior

        // --- Cálculo de la Señal de Control PD ---
        senal_control = (kp * error) + (kd * derivada_error);

        // --- Aplicación de la Señal de Control al Motor ---

        // Limitar la señal de control a un rango manejable para el PWM (ej. +/- 255)
        // Estos límites pueden necesitar ajuste experimental.
        if (senal_control > 255.0) {
            senal_control = 255.0;
        } else if (senal_control < -255.0) {
            senal_control = -255.0;
        }

        // Determinar dirección y valor PWM
        if (fabs(error) < UMBRAL_ESTABILIDAD) { // Si estamos dentro del umbral, detener
             detener_motor();
             pwm_valor = 0;
        } else if (senal_control > 0) { // Señal positiva -> mover adelante
            mover_motor_adelante();
            pwm_valor = (int)senal_control; // Usar la señal de control directamente como PWM
        } else { // Señal negativa -> mover atrás
            mover_motor_atras();
            pwm_valor = (int)(-senal_control); // Usar el valor absoluto para el PWM
        }

        // Asegurar que el valor PWM esté dentro de los límites (0-255)
        if (pwm_valor < 0) {
            pwm_valor = 0;
        } else if (pwm_valor > 255) {
            pwm_valor = 255;
        }

        // Aplicar el valor PWM al registro OCR1A
        OCR1A = pwm_valor;


        // --- Envío de Datos por UART ---
        // Convierte el ángulo actual, error y otros datos a cadenas de texto
        dtostrf(angulo_actual, 6, 2, angulo_str);
        dtostrf(error, 6, 2, error_str);
        // Formatea y envía la cadena por UART, incluyendo la señal de control y el PWM aplicado
        sprintf(buffer_uart, "A%s E%s SC%.2f PWM%d Kp%.2f Kd%.2f AD%.1f\r\n",
                angulo_str, error_str, senal_control, pwm_valor, kp, kd, ANGULO_DESEADO);
        uart_imprimir(buffer_uart);

        // --- Actualización de Variables para la Próxima Iteración ---
        error_anterior = error; // Guarda el error actual para el cálculo derivativo en la siguiente iteración

        // --- Pequeña Pausa ---
        _delay_ms(50); // Pausa para controlar la velocidad del bucle y permitir otras tareas (como UART). Ajustar según sea necesario.

        // La lógica de timeout (stableCounter, noImprovementCounter, generalTimeoutCounter) ha sido eliminada.
        // Los parámetros Kp, Kd y ANGULO_DESEADO ahora se actualizan mediante comandos UART.
    }

    return 0; // Teóricamente nunca se alcanza en un microcontrolador embebido
}
