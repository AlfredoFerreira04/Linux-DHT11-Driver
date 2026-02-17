#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define IOCTL_MAGIC_NUM 'd'

typedef struct {
    uint16_t tempReading;
    uint16_t humReading;
    uint8_t checksum;
} dht_readings;

#define DHT11_PIN_SET   _IOW(IOCTL_MAGIC_NUM, 1, uint32_t)
#define DHT11_READ      _IOR(IOCTL_MAGIC_NUM, 2, dht_readings)

int main() {
    int fd;
    dht_readings data;
    uint32_t new_gpio = 528;

    // Open the device
    fd = open("/dev/dht11", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/dht11");
        return 1;
    }

    // Set GPIO pin
    if (ioctl(fd, DHT11_PIN_SET, &new_gpio) < 0) {
        perror("Failed to set GPIO");
        close(fd);
        return 1;
    }
    printf("GPIO set to %u\n", new_gpio);

    // Read sensor data
    if (ioctl(fd, DHT11_READ, &data) < 0) {
        perror("Failed to read sensor");
        close(fd);
        return 1;
    }

    printf("Temperature: %u.%u°C\n", data.tempReading >> 8, data.tempReading & 0xFF);
    printf("Humidity: %u.%u%%\n", data.humReading >> 8, data.humReading & 0xFF);
    printf("Checksum: %u\n", data.checksum);

    close(fd);
    return 0;
}