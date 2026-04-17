#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#define DRIVER_NAME "ads1115_driver"
#define CLASS_NAME "ads1115"
#define DEVICE_NAME "ads1115"

#define ADS1115_REG_CONVER  0x00
#define ADS1115_REG_CONFIG  0x01
#define ADS1115_REG_LTHRESH 0x02
#define ADS1115_REG_HTHRESH 0x03

#define ADS1115_IOCTL_MAGIC 'A'
#define ADS1115_IOCTL_READ_AN0 _IOR(ADS1115_IOCTL_MAGIC,1,int)
#define ADS1115_IOCTL_READ_AN1 _IOR(ADS1115_IOCTL_MAGIC,2,int)
#define ADS1115_IOCTL_READ_AN2 _IOR(ADS1115_IOCTL_MAGIC,3,int)
#define ADS1115_IOCTL_READ_AN3 _IOR(ADS1115_IOCTL_MAGIC,4,int)

static struct i2c_client *ads1115_client;
static struct class* ads1115_class = NULL;
static struct device* ads1115_device = NULL;
static int major_number;

static int ads1115_read_data(struct i2c_client *client, u8 mux)
{
    u8 buff[2]; // unsign 8bit
    s16 config = (1 << 15)|(mux << 12)|(1 << 9)|(1 << 8)|(4 << 5)|(3 << 0);
    int ret;
    // Combine high and low bytes to form 16-bit values
    buff[0] = (config >> 8) & 0xFF;
    buff[1] = (config & 0xFF);
    // write to reg
    ret = i2c_smbus_write_i2c_block_data(client,ADS1115_REG_CONFIG,2,buff);
    if(ret < 0){
        printk(KERN_ERR "Failed to write to reg \n");
        return ret;
    }

    msleep(10); //wait to conver

    // read data
    if(i2c_smbus_read_i2c_block_data(client,ADS1115_REG_CONVER,sizeof(buff),buff) < 0){
        printk(KERN_ERR "Failed to access the config reg");
        return -EIO;
    }
    s16 value = (buff[0] << 8) | buff[1];
    
    printk(KERN_INFO "Analog at mux %u is : %u \n", mux,value);
    return value;
}

// return all channels
/*static int read_data_all_channels(struct i2c_client *client)
{
    s16 ana_value[4]; // int signed
    s8 ana_channels[4] = {0x04,0x05,0x06,0x07};
    int ret;
    for(int i = 0; i < 4;i++){
        ret = ads1115_read_data(client,ana_channels[i]); // ret = value
        return ret;
        if(ret < 0){
            printk(KERN_ERR "Failed to read channels: %d \n",i);
            return ret;
        }
        ana_value[i] = ret;
        printk(KERN_INFO "Analog channels %d is : %u",i,ana_value);
    }
    return 0;
}*/

static long ads1115_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    int data;

    switch (cmd)
    {
    case ADS1115_IOCTL_READ_AN0:
        data = ads1115_read_data(ads1115_client,4); // 0x04 (mux)
        break;
    case ADS1115_IOCTL_READ_AN1:
        data = ads1115_read_data(ads1115_client,5); // 0x05 (mux)
        break;
    
    case ADS1115_IOCTL_READ_AN2:
        data = ads1115_read_data(ads1115_client,6); // 0x06 (mux)
        break; 
    
    case ADS1115_IOCTL_READ_AN3:
        data = ads1115_read_data(ads1115_client,7); // 0x07 (mux)
        break;
    
    default:
        return -EINVAL;
    }
    
    if(copy_to_user((int __user *)arg, &data, sizeof(data)))
    {
        return -EFAULT;
    }
    return 0;
}

static int ads1115_open(struct inode *inodep, struct file* filep)
{
    printk(KERN_INFO "ADS1115 device open \n");
    return 0;
}

static int ads1115_release(struct inode *inodep,struct file *filep)
{
    printk(KERN_INFO "ADS1115 device release \n");
    return 0;
}

static struct file_operations fops = {
    .open = ads1115_open,
    .release = ads1115_release,
    .unlocked_ioctl = ads1115_ioctl,
};

static int ads1115_probe(struct i2c_client *client)
{
    ads1115_client = client;

    // create char device
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if(major_number < 0){
        printk(KERN_ERR "Failed to register a major number \n");
        return major_number;
    }

    ads1115_class = class_create(CLASS_NAME);
    if(IS_ERR(ads1115_class)){
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to register device class \n");
    }

    ads1115_device = device_create(ads1115_class, NULL,MKDEV(major_number,0), NULL, DEVICE_NAME);
    if(IS_ERR(ads1115_device)){
        class_destroy(ads1115_class);
        printk(KERN_ERR "Failed to create device \n");
        return PTR_ERR(ads1115_device);
    }

    printk(KERN_INFO "ADS1115 driver installer \n");
    return 0;
}

static void ads1115_remove(struct i2c_client *client)
{
    device_destroy(ads1115_class, MKDEV(major_number,0));
    class_unregister(ads1115_class);
    class_destroy(ads1115_class);
    unregister_chrdev(major_number,DEVICE_NAME);

    printk(KERN_INFO "ADS1115 has been removed \n");
}

static const struct of_device_id ads1115_of_match[] = {
    {
        .compatible = "Texas Instrument, ads1115",},
        { },
};

MODULE_DEVICE_TABLE(of,ads1115_of_match);

static struct i2c_driver ads1115_driver = {
    .driver = {
        .name = DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table  = of_match_ptr(ads1115_of_match),
    },
    .probe = ads1115_probe,
    .remove = ads1115_remove,
};

static int __init ads1115_init(void){
    printk(KERN_INFO " Initializing ads1115 driver \n");
    return i2c_add_driver(&ads1115_driver);
}

static void  __exit ads1115_exit(void){
    printk(KERN_INFO " Exiting ads1115 driver \n");
    i2c_del_driver(&ads1115_driver);
}

module_init(ads1115_init);
module_exit(ads1115_exit);

MODULE_AUTHOR("Pham Thai Hung");
MODULE_DESCRIPTION("ADS1115 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("GPL");
