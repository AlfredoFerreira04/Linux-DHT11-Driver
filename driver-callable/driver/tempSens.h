#include "linux/ioctl.h"
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>  
#include <linux/gpio/consumer.h>
#include <linux/irqflags.h>
#include <linux/ioctl.h>
#include <linux/types.h>

#define GPIO_NUMBER      528
#define TEMP_FILE_PATH   "TEMP_SENSOR_GPIO"
#define DHT11_TIMEOUT_US 200

#define IOCTL_MAGIC_NUM 'd'

// Temperature and Humidity values passed as integers for simplicity,
// this is possible because these values ALWAYS have 8 integer bits and 8 decimal bits
typedef struct{
    u16 tempReading;
    u16 humReading;
    u8  checksum;
} dht_readings;


#define DHT11_PIN_SET   _IOW(IOCTL_MAGIC_NUM, 1, u32)
#define DHT11_READ      _IOR(IOCTL_MAGIC_NUM, 2, dht_readings)


static int wait_for_level(int level, int timeout_us);
long dht11_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static dht_readings read_data(void);

