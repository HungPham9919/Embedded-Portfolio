#include "i2c.h"
i2c_master_bus_handle_t i2c1_bus_handle;

esp_err_t i2c1_master_init(void){
    i2c_master_bus_config_t i2c1_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C1_MASTER_PORT,
        .glitch_ignore_cnt = 7,
        .scl_io_num = I2C1_MASTER_SCL_IO,
        .sda_io_num = I2C1_MASTER_SDA_IO,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c1_cfg,&i2c1_bus_handle);
    if(ret != ESP_OK){
        ESP_LOGE("I2C1","Failed to create I2C1 bus %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI("I2C1","I2C1 bus initialized succesfully");
    return ESP_OK;
}

void check_address(void){
    if(bus_handle == NULL){
        ESP_LOGW("Bus","bus is not initialized");
        return;
    }

    int found = 0;
    for(uint16_t addr = 1; addr < 127;addr++){
        if(i2c_master_probe(bus_handle,addr,pdMS_TO_TICKS(100)) == ESP_OK){
            found++;
            ESP_LOGI("Address","Found %d device , address 0x%02X",found,addr);
        }
        vTaskDelay(10);
    }
    if(found == 0){
        ESP_LOGI("Address","Not found any devices");
    }
    else {
        ESP_LOGI("Address","Scan completed, found %d device",found);
    }
}

void i2c1_check_address(void){
    if(i2c1_bus_handle == NULL){
        ESP_LOGW("Bus","bus is not initialized");
        return;
    }

    int found = 0;
    for(uint16_t addr = 1; addr < 127;addr++){
        if(i2c_master_probe(i2c1_bus_handle,addr,pdMS_TO_TICKS(100)) == ESP_OK){
            found++;
            ESP_LOGI("Address","Found %d device , address 0x%02X",found,addr);
        }
        vTaskDelay(10);
    }
    if(found == 0){
        ESP_LOGI("Address","Not found any devices");
    }
    else {
        ESP_LOGI("Address","Scan completed, found %d device",found);
    }
}