
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/fcntl.h>
#include <pico/unistd.h>
#include <pico/string.h>
#include <pico/wiringPi.h>

#define BUFFER_SIZE 256
#define ANALOG_PIN 0
#define ANALOG_PIN2 0


int main() {

SensorTemperatura();
SensorHumedad();
SensorLuz();



}

int SensorTemperatura(){
    int fd;
    char buffer[BUFFER_SIZE];

    // Abrir el archivo del sensor
    fd = open("/sys/bus/w1/devices/28-*/w1_slave", O_RDONLY);
    if (fd == -1) {
        perror("Error al abrir el archivo del sensor");
        return 1;
    }

    // Leer el archivo del sensor
    if (read(fd, buffer, BUFFER_SIZE) == -1) {
        perror("Error al leer el archivo del sensor");
        close(fd);
        return 1;
    }

    // Extraer la temperatura del buffer
    char *temperature_start = strstr(buffer, "t=");
    if (temperature_start == NULL) {
        printf("No se pudo obtener la temperatura\n");
        close(fd);
        return 1;
    }
    float temperature = strtof(temperature_start + 2, NULL) / 1000;

    // Imprimir la temperatura
    printf("Temperatura: %.2f grados Celsius\n", temperature);

    // Cerrar el archivo del sensor
    close(fd);

    return 0;


}

int SensorHumedad(){

    wiringPiSetup();
    pinMode(ANALOG_PIN, INPUT);

    while (1) {
        int sensorValue = analogRead(ANALOG_PIN);
        float humidity = map(sensorValue, 0, 1023, 0, 100);

        printf("Humedad del suelo: %.2f%%\n", humidity);

        delay(1000);
    }

    return 0;


}


int SensorLuz(){

wiringPiSetup();
    pinMode(ANALOG_PIN, INPUT);

    while (1) {
        int sensorValue = analogRead(ANALOG_PIN);

        printf("Intensidad de luz: %d\n", sensorValue);

        delay(1000);
    }

    return 0;




}