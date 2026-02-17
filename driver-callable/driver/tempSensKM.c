#include "tempSens.h"
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>

u16 set_gpio_number = GPIO_NUMBER;
static struct gpio_desc *dht11_gpio;

// Character device structures
static dev_t dev_num;
static struct cdev dht11_cdev;
static struct class *dht11_class;

long dht11_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
    dht_readings read;
    switch(cmd){
        case DHT11_PIN_SET:
            if( copy_from_user(&set_gpio_number, (u16*) arg, sizeof(set_gpio_number)) ){
                printk(KERN_ERR "UNABLE TO RECEIVE NEW GPIO %d!\n", (u16*) arg);
            }else{
                printk(KERN_INFO "GPIO Updated to %d", set_gpio_number);
            }
            break;
        case DHT11_READ:
            read = read_data();
            if( copy_to_user((dht_readings*) arg, &read, sizeof(read)) ){
                printk(KERN_ERR "UNABLE TO SEND DHT READINGS %d!\n", (u16*) arg);
            }else{
                printk(KERN_INFO "GPIO Readings Sent!");
            }
            break;
    }
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = dht11_ioctl,
};

static int wait_for_level(int level, int timeout_us)
{
    while (gpiod_get_raw_value(dht11_gpio) != level) {
        if (--timeout_us <= 0)
            return -ETIMEDOUT;
        udelay(1);
    }
    return 0;
}

static dht_readings read_data(void)
{
    dht_readings read;
    u8 arr[5];
    u8 current_byte;
    uint64_t flags;


    local_irq_save(flags);

    gpiod_direction_output(dht11_gpio, 0);                          
    mdelay(20);
    gpiod_set_raw_value(dht11_gpio, 1);                             
    udelay(30);
    gpiod_direction_input(dht11_gpio);                              
    wait_for_level(0, DHT11_TIMEOUT_US);
    udelay(75);
    wait_for_level(1, DHT11_TIMEOUT_US);
    udelay(75);

    for (int i = 0; i < 5; ++i) {
        current_byte = 0;
        for (int j = 0; j < 8; j++) {
            wait_for_level(0, DHT11_TIMEOUT_US);
            udelay(45);
            wait_for_level(1, DHT11_TIMEOUT_US);
            udelay(29);

            if (gpiod_get_raw_value(dht11_gpio) == 1) {
                current_byte += 1;
                udelay(30);
            }

            if (j != 7) {current_byte = current_byte << 1;}
        }
        arr[i] = current_byte;
    }

    local_irq_restore(flags);
    
    u8 checksum = arr[0] + arr[1] + arr[2] + arr[3];
    printk(KERN_INFO "--- Temperature Sensor Info ---\n");
    printk(KERN_INFO "Humidity : %d.%d\n", arr[0], arr[1]);
    printk(KERN_INFO "Temperature : %d.%d\n", arr[2], arr[3]);
    printk(KERN_INFO "Checksum received: %d | Checksum calculated: %d\n", arr[4], checksum);
    
    read.humReading = (arr[0] << 8) | arr[1];
    read.tempReading = (arr[2] << 8) | arr[3];
    read.checksum = arr[4];  // ← fixed index
    return read;
}

static int __init temperature_sensor_init(void)
{
    if (gpio_request(set_gpio_number, TEMP_FILE_PATH) != 0) {
        printk(KERN_ERR "Failed to request GPIO %d\n", set_gpio_number);
        return -1;
    }
    
    dht11_gpio = gpio_to_desc(set_gpio_number);
    if (!dht11_gpio) {
        printk(KERN_ERR "Failed to get GPIO descriptor\n");
        gpio_free(set_gpio_number);
        return -ENODEV;
    }
    
    // Allocate device number dynamically
    if (alloc_chrdev_region(&dev_num, 0, 1, "dht11") < 0) {
        printk(KERN_ERR "Failed to allocate device number\n");
        gpio_free(set_gpio_number);
        return -1;
    }
    
    // Create device class
    dht11_class = class_create("dht11");
    if (IS_ERR(dht11_class)) {
        printk(KERN_ERR "Failed to create class\n");
        unregister_chrdev_region(dev_num, 1);
        gpio_free(set_gpio_number);
        return PTR_ERR(dht11_class);
    }
    
    // Create device file /dev/dht11
    if (!device_create(dht11_class, NULL, dev_num, NULL, "dht11")) {
        printk(KERN_ERR "Failed to create device\n");
        class_destroy(dht11_class);
        unregister_chrdev_region(dev_num, 1);
        gpio_free(set_gpio_number);
        return -1;
    }
    
    // Initialize and add character device
    cdev_init(&dht11_cdev, &fops);
    if (cdev_add(&dht11_cdev, dev_num, 1) < 0) {
        printk(KERN_ERR "Failed to add cdev\n");
        device_destroy(dht11_class, dev_num);
        class_destroy(dht11_class);
        unregister_chrdev_region(dev_num, 1);
        gpio_free(set_gpio_number);
        return -1;
    }
    
    printk(KERN_INFO "DHT11 device created at /dev/dht11\n");
    return 0;
}

static void __exit temperature_sensor_exit(void)
{
    cdev_del(&dht11_cdev);
    device_destroy(dht11_class, dev_num);
    class_destroy(dht11_class);
    unregister_chrdev_region(dev_num, 1);
    gpio_free(set_gpio_number);
    printk(KERN_INFO "DHT11 device removed\n");
}

module_init(temperature_sensor_init);
module_exit(temperature_sensor_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("AlfredoFerreira04");
MODULE_DESCRIPTION("Temp Sensor Driver");
MODULE_VERSION("1.0");