#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>  
#include <linux/gpio/consumer.h>
#include <linux/irqflags.h>
#include <linux/types.h>
#include <linux/ioctl.h>

#define GPIO_NUMBER      528
#define TEMP_FILE_PATH   "TEMP_SENSOR_GPIO"
#define DHT11_TIMEOUT_US 200

static struct task_struct *temp_thread;
static struct gpio_desc *dht11_gpio;                                   
static int wait_for_level(int level, int timeout_us)
{
    while (gpiod_get_raw_value(dht11_gpio) != level) {                  
        if (--timeout_us <= 0)
            return -ETIMEDOUT;
        udelay(1);
    }
    return 0;
}

static int temp_thread_fn(void *data)
{
    while (!kthread_should_stop()) {
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
                    udelay(40);
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
        ssleep(5);
    }
    return 0;
}

static int __init temperature_sensor_init(void)
{

    if (gpio_request(GPIO_NUMBER, TEMP_FILE_PATH) != 0) {              
        printk(KERN_ERR "Failed to request GPIO %d\n", GPIO_NUMBER);
        return -1;
    }
    dht11_gpio = gpio_to_desc(GPIO_NUMBER);                             
    if (!dht11_gpio) {                                                  
        printk(KERN_ERR "Failed to get GPIO descriptor\n");            
        return -ENODEV;                                                 
    }                                                                   

    temp_thread = kthread_run(temp_thread_fn, NULL, "temp_sensor");
    if (IS_ERR(temp_thread)) {
        gpio_free(dht11_gpio);                                         
        return PTR_ERR(temp_thread);
    }

    return 0;
}

static void __exit temperature_sensor_exit(void)
{
    if (temp_thread)
        kthread_stop(temp_thread);
    gpio_free(GPIO_NUMBER);                                             
}

module_init(temperature_sensor_init);
module_exit(temperature_sensor_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("AlfredoFerreira04");
MODULE_DESCRIPTION("Temp Sensor Driver");
MODULE_VERSION("1.0");