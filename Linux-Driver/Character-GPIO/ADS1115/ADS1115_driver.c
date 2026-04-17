#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h> //msleep
#define DRIVER_NAME "ads1115_driver"

//registor
#define ADS1115_ADDR 0x48
#define ADS1115_REG_CONVER  0x00
#define ADS1115_REG_CONFIG  0x01
#define ADS1115_REG_LTHRESH 0x02
#define ADS1115_REG_HTHRESH 0x03

static struct i2c_client *ads1115_client;
    /*
            bit 432 dont care
    OS = 1 // single version
    Muốn đo từng kênh thì đo nó với đất
    MUX = 100 -> AN0
    PGA = 000 (2048) -> amplifier gain
    MODE : singgle shot 1
    Data Rate: 100 = 128 (default)
    Bit [1:0]  = 11 
    config = 1100 0011 1000 0011 = C183
    */
static int ads1115_config(struct i2c_client *client)
{
    // gửi tín hiệu lên i2c trước ví dụ: 0x01 ->config || Tự đọc 2 byte
    int acc_cfg = i2c_smbus_read_word_data(client,ADS1115_REG_CONFIG);
    if(acc_cfg < 0){
        printk(KERN_ERR "Cannot access to config_reg");
        return -EIO;
    }
    //Ghi cấu hình vào reg
    s16 config = 0xC183; // cấu hình trước
    u8 bufconfig[2]; // config
    bufconfig[0] = (config >> 8) & 0xFF; // MSB
    bufconfig[1] = (config & 0xFF); // LSB
    int ret;
    ret = i2c_smbus_write_i2c_block_data(client,ADS1115_REG_CONFIG,2,bufconfig);
    if(ret < 0){
        printk(KERN_ERR "Failed to write to reg \n");
        return ret;
    }
    // double check
    if(i2c_smbus_read_i2c_block_data(client,ADS1115_REG_CONFIG,sizeof(bufconfig),bufconfig) < 0){
        printk(KERN_ERR "Failed to read the config reg");
        return -EIO;
    } 
    return 0;
}
// read channels
static int ads1115_read_data(struct i2c_client *client, u8 mux)
{
    u8 buff[2]; // unsign 8bit
    s16 config = 0xC183;
    int ret;
    // Combine high and low bytes to form 16-bit values
    buff[0] = (config >> 8) & 0xFF;
    buff[1] = (config & 0xFF);
    // write to reg
    ret = i2c_smbus_write_i2c_block_data(client,ADS1115_REG_CONFIG,2,buff);
    if(ret < 0){
        printk(KERN_ERR "Failed to write to reg");
        return ret;
    }

    msleep(10); //wait to conver

    // read data
    if(i2c_smbus_read_i2c_block_data(client,ADS1115_REG_CONVER,sizeof(buff),buff) < 0){
        printk(KERN_ERR "Failed to access the config reg");
        return -EIO;
    }
    s16 value = (buff[0] << 8) | buff[1];
    
    printk(KERN_INFO "Analog at mux %d is : %d \n", mux,value);
    return value;
}
 // in version linux 5.5 can use static int ads1115_probe(struct i2c_client *client, const struct i2c_device_id *id)
static int ads1115_probe(struct i2c_client *client)
{
    ads1115_client = client; // Lưu lại để các hàm khác dễ dùng
    int ret;
    // check the config
    ret = ads1115_config(client); // channel 0
    if(ret < 0){
        printk(KERN_INFO "Failed to configure ads \n");
        return ret;
    }
    // Read data from ads
    ret = ads1115_read_data(client,0); // channel 0
    if(ret < 0){
        printk(KERN_ERR "Failed to read data \n");
        return ret;
    }

    printk(KERN_INFO "ADS1115 driver has been installed \n");
    return 0;
}
static const struct i2c_device_id ads1115_id[] = {
    {"ads1115",0},
    {}
}; 

MODULE_DEVICE_TABLE(i2c,ads1115_id);

static void ads1115_remove(struct i2c_client *client){
    printk(KERN_INFO "ADS1115 driver has been removed \n");

    // all clean
}

static struct i2c_driver ads1115_driver =
{
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe = ads1115_probe,
    .remove = ads1115_remove,
    .id_table = ads1115_id,
};

// Initialize the i2c driver
static int __init ads1115_init(void){
    printk(KERN_INFO "Initializing ADS1115 driver \n");
    return i2c_add_driver(&ads1115_driver); 
}

static void __exit ads1115_exit(void){
    printk(KERN_INFO "Exiting ADS1115 driver \n");
    i2c_del_driver(&ads1115_driver);
}

module_init(ads1115_init);
module_exit(ads1115_exit);

MODULE_AUTHOR("PHAM THAI HUNG");
MODULE_DESCRIPTION("ADS1115 I2C Client Driver");
MODULE_LICENSE("GPL");
