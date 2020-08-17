#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/sysfs.h>
#include <linux/sysfs.h>

#define HIGH_STATE 1
#define LOW_STATE 0

char address_device;

const struct Regs{
        const int mode1;
        const int mode2;
        const int pwm0;
        const int pwm1;
        const int pwm2;
        const int pwm3;
        int const pwm4;
        int const pwm5;
        int const pwm6;
        int const pwm7;
        int const pwm8;
        int const pwm9;
        int const pwm10;
        int const pwm11;
        int const pwm12;
        int const pwm13;
        int const pwm14;
        int const pwm15;
        int const grppwm;
        int const grpfreq;
        int const ledOut0;
        int const ledOut1;
        int const ledOut2;
        int const ledOut3;
        int const subAdr1;
        int const subAdr2;
        int const subAdr3;
} ledsReg = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
             11, 12, 13, 14, 15, 16, 17, 18, 19,
             20, 21, 22, 23, 24, 25, 26 };

struct pca9622leds_params {
    ktime_t timeout_group1;       // usage ->  timeout_latch = ktime_set( 0, 1000 );
    ktime_t timeout_group2;
    unsigned int r1; unsigned int g1; unsigned int b1;
    unsigned int r2; unsigned int g2; unsigned int b2;
    unsigned int r3; unsigned int g3; unsigned int b3;
    unsigned int r4; unsigned int g4; unsigned int b4;
    unsigned int r5; unsigned int g5; unsigned int b5;

    unsigned int r1_max; unsigned int g1_max; unsigned int b1_max;
    unsigned int r2_max; unsigned int g2_max; unsigned int b2_max;
    unsigned int r1_min; unsigned int g1_min; unsigned int b1_min;
    unsigned int r2_min; unsigned int g2_min; unsigned int b2_min;
    char gain_c;
};

struct pca9622leds_data;

struct pca9622leds_data {
    struct i2c_client *client;
    struct device *dev;
    struct input_dev *dev_in;
    struct gpio_desc *irqpin;
    struct hrtimer hrt_group1;
    struct hrtimer hrt_group2;
    struct work_struct work_group1;
    struct work_struct work_group2;
    struct pca9622leds_params params;
};

static int pca9622leds_i2c_write(struct device *dev, unsigned int reg,
                                unsigned int val);
static int pca9622leds_i2c_read(struct device *dev);

/* ------------------------------- */

/* ATTRIBUTE brightness */
/*!
 * @brief Attribute to set_brightness,
 * @param - *buf:
 * [0] - group of the leds
 * [123] - colors rgb, max value
 */
static ssize_t brightness_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t pos, size_t count)
{
    struct device *dev = kobj_to_dev(kobj);
    struct pca9622leds_data *leddata;

    if(count != 4) {
        dev_printk(KERN_ERR, dev, "Brightness attribute: false count buffer,"
                                  " must be 4, now its %d.\n", count);
        return -EINVAL;
    }
    unsigned int led_nr = (unsigned int)buf[0];

    switch (led_nr) {
    case 1:
        leddata->params.r1 = buf[1];
        leddata->params.g1 = buf[2];
        leddata->params.b1 = buf[3];

        pca9622leds_i2c_write(dev, ledsReg.pwm0, leddata->params.r1);
        pca9622leds_i2c_write(dev, ledsReg.pwm1, leddata->params.g1);
        pca9622leds_i2c_write(dev, ledsReg.pwm2, leddata->params.b1);
        break;
    case 2:
        leddata->params.r2 = buf[1];
        leddata->params.g2 = buf[2];
        leddata->params.b2 = buf[3];

        pca9622leds_i2c_write(dev, ledsReg.pwm3, leddata->params.r2);
        pca9622leds_i2c_write(dev, ledsReg.pwm4, leddata->params.g2);
        pca9622leds_i2c_write(dev, ledsReg.pwm5, leddata->params.b2);
        break;
    case 3:
        leddata->params.r3 = buf[1];
        leddata->params.g3 = buf[2];
        leddata->params.b3 = buf[3];

        pca9622leds_i2c_write(dev, ledsReg.pwm6, leddata->params.r3);
        pca9622leds_i2c_write(dev, ledsReg.pwm7, leddata->params.g3);
        pca9622leds_i2c_write(dev, ledsReg.pwm8, leddata->params.b3);
        break;
    case 4:
        leddata->params.r4 = buf[1];
        leddata->params.g4 = buf[2];
        leddata->params.b4 = buf[3];

        pca9622leds_i2c_write(dev, ledsReg.pwm9, leddata->params.r4);
        pca9622leds_i2c_write(dev, ledsReg.pwm10, leddata->params.g4);
        pca9622leds_i2c_write(dev, ledsReg.pwm11, leddata->params.b4);
        break;
    case 5:
        leddata->params.r5 = buf[1];
        leddata->params.g5 = buf[2];
        leddata->params.b5 = buf[3];

        pca9622leds_i2c_write(dev, ledsReg.pwm12, leddata->params.r5);
        pca9622leds_i2c_write(dev, ledsReg.pwm13, leddata->params.g5);
        pca9622leds_i2c_write(dev, ledsReg.pwm14, leddata->params.b5);
        break;
    default:
        dev_printk(KERN_ERR, dev, "Brightness : false group.\n");
        return -EINVAL;
        break;
    }
    dev_printk(KERN_ERR, dev, "Led: %d Brightness.\n", led_nr);
    return count;
}
static ssize_t brightness_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t pos, size_t count) {
    return -EINVAL;
}
static BIN_ATTR_RW(brightness, 0);

/* --------------------------------------- */

/* ATTRIBUTE set i2c */
/*!
 * @brief Attribute to set_brightness max,
 * @param - *buf:
 * [0] - group of the leds
 * [123] - colors rgb, max value
 */
static ssize_t seti2c_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t pos, size_t count) {

    unsigned int reg;
    unsigned int val;
    struct device *dev = kobj_to_dev(kobj);
    if(count != 2)
        return -EINVAL;
    if(pos != 0)
        return -EINVAL;
    reg = (unsigned int)buf[0];
    val = (unsigned int)buf[1];
    pca9622leds_i2c_write(dev, reg, val);
    return count;
}
static ssize_t seti2c_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t pos, size_t count) {
    return -EINVAL;
}
static BIN_ATTR_RW(seti2c, 0);

/* --------------------------------------- */

/* ATTRIBUTE read i2c */
/*!
 * @brief Attribute to set_brightness max,
 * @param - *buf:
 * [0] - group of the leds
 * [123] - colors rgb, max value
 */
static ssize_t readi2c_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t pos, size_t count) {

    unsigned int reg;
    unsigned int val;
    struct device *dev = kobj_to_dev(kobj);
    if(count != 2)
        return -EINVAL;
    if(pos != 0)
        return -EINVAL;
    /* TODO: */
    return buf;
}
static ssize_t readi2c_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t pos, size_t count) {
    return -EINVAL;
}
static BIN_ATTR_RW(readi2c, 0);


/* --------------------------------------------------------------------------------------------*/

/* Init module printer driver */

/* Functions */
static int pca9622leds_i2c_write(struct device *dev, unsigned int reg,
                                unsigned int val) {
    struct i2c_client *client = to_i2c_client(dev);
    return i2c_smbus_write_byte_data(client, reg, val);
}

static int pca9622leds_i2c_read(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    return i2c_smbus_read_byte(client);
}
/* ------------------------------------------------------------------------------------------- */

static int pca9622leds_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct pca9622leds_data *leddata;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
            dev_printk(KERN_ERR, &client->dev, "I2C check functionality failed.\n");
            return -ENXIO;
    }

    leddata = devm_kzalloc(&client->dev, sizeof(struct pca9622leds_data), GFP_KERNEL);
    if(!leddata) return -ENOMEM;

    char address_device = client->addr;

    dev_printk(KERN_INFO, leddata->dev,"I2C Address: 0x%02x\n", client->addr);
    dev_printk(KERN_INFO, leddata->dev,"CHAR Address: %x \n", address_device);
    leddata->client = client;
    /* Set client to struct leddata */
    i2c_set_clientdata(client, leddata);
    leddata->dev = &client->dev;

    pca9622leds_i2c_write(leddata->dev, ledsReg.mode1, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut0, 0xAA);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut1, 0xAA);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut2, 0xAA);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut3, 0xAA);
    /* Turn off leds */
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm0, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm1, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm2, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm3, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm4, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm5, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm6, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm7, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm8, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm9, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm10, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm11, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm12, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm13, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm14, 0x00);
    pca9622leds_i2c_write(leddata->dev, ledsReg.pwm15, 0x00);

    device_create_bin_file(leddata->dev, &bin_attr_brightness);
    dev_printk(KERN_INFO, leddata->dev, "brightness bin attribute installing\n");
    device_create_bin_file(leddata->dev, &bin_attr_seti2c);
    dev_printk(KERN_INFO, leddata->dev, "seti2c bin attribute installing\n");
    device_create_bin_file(leddata->dev, &bin_attr_readi2c);
    dev_printk(KERN_INFO, leddata->dev, "readi2c bin attribute installing\n");

    dev_printk(KERN_INFO, leddata->dev, "pca9622leds: probe completed\n");
    return 0;
}

static int pca9622leds_remove(struct i2c_client *client) {

    struct pca9622leds_data *leddata = i2c_get_clientdata(client);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut0, 0);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut1, 0);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut2, 0);
    pca9622leds_i2c_write(leddata->dev, ledsReg.ledOut3, 0);

    device_remove_bin_file(leddata->dev, &bin_attr_brightness);
    dev_printk(KERN_INFO, leddata->dev, "brightness bin attribute remove\n");
    device_remove_bin_file(leddata->dev, &bin_attr_seti2c);
    dev_printk(KERN_INFO, leddata->dev, "seti2c bin attribute remove\n");
    device_remove_bin_file(leddata->dev, &bin_attr_readi2c);
    dev_printk(KERN_INFO, leddata->dev, "readi2c bin attribute remove\n");
    input_unregister_device(leddata->dev_in);
    
    pr_info("pca9622leds: Remove keyboard device \n");
    return 0;
}
static const struct i2c_device_id pca9622leds_i2c_ids[] = {
        { "pca9622leds", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, pca9622leds_i2c_ids);

static const struct of_device_id pca9622leds_of_match[] = {
        { .compatible = "st,pca9622" },
        { }
};
MODULE_DEVICE_TABLE(of, pca9622leds_of_match);
/* 
name 		-	Name of the device driver.
of_match_table 	-  	The open firmware table.
probe 		-	Called to query the existence of a specific device,
whether this driver can work with it, and bind the driver to a specific device.
*/
static struct i2c_driver pca9622leds_i2c_driver = {
	.driver = {
                .name = "pca9622leds",
                .of_match_table = pca9622leds_of_match,
                .owner = THIS_MODULE,
	},
        .id_table = pca9622leds_i2c_ids,
        .probe = pca9622leds_probe,
        .remove = pca9622leds_remove
};
module_i2c_driver(pca9622leds_i2c_driver);

MODULE_AUTHOR("STRING");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PCA9622leds button driver");

