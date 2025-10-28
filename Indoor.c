// ---------------------------- Importación de librerías
#include <stdio.h> 
#include <string.h>
#include "pico/stdlib.h" 
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "blink.pio.h"
#include "onewire_library/onewire_library.h"  // Librería 1-Wire con estructura OW
#include "ow_rom.h"                           // Comandos ROM del protocolo 1-Wire
#include "ds18b20.h"                          // Comandos específicos del sensor DS18B20
// ----------------------------

// ---------------------------- Definición de pines
#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define GPIO 2 // Pin donde está conectado el DS18B20

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define LDR_PIN 27  // Pin conectado al AO del módulo LDR

// ----------------------------

// Estructura global para manejar el bus 1-Wire
OW ow;

// ---------------------------- Función para parpadeo con PIO
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    printf("Parpadeando el pin %d a %d Hz\n", pin, freq);
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
// ----------------------------


// ---------------------------- Lectura de temperatura con estructura OW
float SensorTemperatura() {
    uint8_t scratchpad[9];

    // 1. Reset del bus
    if (!ow_reset(&ow)) {
        printf("Sensor no detectado\n");
        return -1000.0f;
    }

    // 2. SKIP ROM
    ow_send(&ow, OW_SKIP_ROM);

    // 3. CONVERT_T
    ow_send(&ow, DS18B20_CONVERT_T);

    // 4. Esperar conversión (~750ms para 12 bits)
    sleep_ms(750);

    // 5. Reset otra vez
    if (!ow_reset(&ow)) {
        printf("Sensor no detectado tras conversión\n");
        return -1000.0f;
    }

    // 6. SKIP ROM
    ow_send(&ow, OW_SKIP_ROM);

    // 7. READ SCRATCHPAD
    ow_send(&ow, DS18B20_READ_SCRATCHPAD);

    // 8. Leer 9 bytes del scratchpad
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = ow_read(&ow);
    }

    // 9. Extraer temperatura de los primeros 2 bytes
    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    float temp_celsius = raw_temp / 16.0f;

    return temp_celsius;
}
// ----------------------------

// ---------------------------- Lectura de humedad con ADC
float SensorHumedad() {
    // Leer valor analógico del canal 0 (GPIO 26)
    adc_select_input(0);
    uint16_t raw = adc_read();

    // Convertir a porcentaje (0–100%)
    float humedad = (raw / 4095.0f) * 100.0f;

    return humedad;
}
// ----------------------------

// ---------------------------- Lectura de luz con ADC
float SensorLuz() {
    adc_select_input(1);  // Canal 1 corresponde al GPIO 27
    uint16_t raw = adc_read();

    // Convertir a porcentaje (0–100%)
    float luz = (raw / 4095.0f) * 100.0f;

    return luz;
}
// ----------------------------


// ---------------------------- Función principal
int main() {
    stdio_init_all();

    // UART: inicializa el puerto UART1 para comunicación serie (ej. con PC o Bluetooth)
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // GPIO: configura el pin 2 como entrada con resistencia pull-up (ej. para botones o sensores digitales)
    gpio_init(GPIO);
    gpio_set_dir(GPIO, GPIO_IN);
    gpio_pull_up(GPIO);

    // I2C: configura el bus I2C0 a 400 kHz y activa resistencias pull-up en SDA y SCL
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicialización del ADC para humedad y luz
    gpio_init(26);  // Humedad (canal 0)
    gpio_init(LDR_PIN);  // Luz (canal 1)
    adc_init();

    // PIO: carga el programa PIO para parpadeo de LED y lo ejecuta en el pin por defecto o el GPIO 6
    PIO pio = pio0;
    uint offset_blink = pio_add_program(pio, &blink_program);
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset_blink, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset_blink, 6, 3);
    #endif

    // Inicialización del bus 1-Wire para el DS18B20
    uint offset_ow = pio_add_program(pio, &onewire_program);
    if (!ow_init(&ow, pio, offset_ow, GPIO)) {
        printf("Error al inicializar el bus 1-Wire\n");
        return 1;
    }

    // Bucle principal
    while (true) {
        // Lectura de temperatura
        float temp = SensorTemperatura();
        if (temp > -100.0f) {
            printf("Lectura OK: %.2f °C\n", temp);
        } else {
            printf("Error de lectura\n");
        }

        // Lectura de humedad 
        float humedad = SensorHumedad();
        if (humedad >= 0.0f && humedad <= 100.0f) {
            printf("Humedad del suelo: %.2f %%\n", humedad);
        } else {
            printf("Error de lectura de humedad\n");
        }

        // Lectura de luz
        float luz = SensorLuz();
        if (luz >= 0.0f && luz <= 100.0f) {
            printf("Intensidad lumínica: %.2f %%\n", luz);
        } else {
            printf("Error de lectura de luz\n");
        }

        sleep_ms(1000);
    }
}
// ----------------------------
