#include "BMI160.h"

i2c_master_dev_handle_t bmi160_handle;
volatile uint32_t last_sensor_time = 0;
float delta_t = 0;
final_data sensor_val;
TaskHandle_t bmi160_task = NULL;


#define RAD_TO_DEG 57.295779513f
#define ALPHA 0.96f

float gx_os = 0, gy_os = 0, gz_os = 0;
float ax_os = 0, ay_os = 0, az_os = 0;

float roll_acc = 0.0f, pitch_acc = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

struct BMI160_sensitivity sens = {
    .acc_cfg = 0x28,
    .acc_range = 0x05,
    .gyro_cfg = 0x28,
    .gyro_range = 0x01,
};

float gyro_sens,acc_sens;
uint8_t data_array[15];
Raw_data bmi160_data;

void IRAM_ATTR cal_flight_angle(final_data *sensor_val, float dt){

    roll_acc = atan2f(sensor_val->ay, sensor_val->az) *RAD_TO_DEG;
    pitch_acc = atan2f(-sensor_val->ax, sqrtf(sensor_val->ay*sensor_val->ay + sensor_val->az*sensor_val->az)) * RAD_TO_DEG;

    //Complementary filter

    roll = ALPHA *(roll + sensor_val->gx*dt) + (1.0f - ALPHA) * roll_acc;
    pitch = ALPHA * (pitch + sensor_val->gy*dt) + (1.0f - ALPHA)*  pitch_acc;
    yaw += sensor_val->gz *dt; // drift

    if(yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    printf("R_a: %.2f | P_a : %.2f \n",roll_acc, pitch_acc);
    printf("R: %.2f | P: %.2f | Y: %.2f\n", roll, pitch, yaw);

}

void sensitivity(void){
    switch (sens.gyro_range)
    {
    case 0x00:
        gyro_sens = 16.4f;
        break;
    case 0x01:
        gyro_sens = 32.8f;
        break;
    case 0x02:
        gyro_sens = 65.6f;
        break;
    case 0x03:
        gyro_sens = 131.2f;
        break;
    case 0x04:
        gyro_sens = 262.4f;
        break;
    default:
        break;
    }

    switch (sens.acc_range)
    {
    case 0x03:
        acc_sens = 16384;
        break;
    case 0x05:
        acc_sens = 8192;
        break;
    case 0x08:
        acc_sens = 4096;
        break;
    case 0x0C:
        acc_sens = 2048;
        break;
    default:
        break;
    }
}

esp_err_t BMI160_Init(void){
    if(i2c1_bus_handle == NULL){
        ESP_LOGE("Bus","Bus is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = BMI160_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ, // 400Khz
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c1_bus_handle,&cfg,&bmi160_handle));

    return ESP_OK;
}

esp_err_t BMI160_CMD(uint8_t cmd, uint8_t value){
    uint8_t buff[2] = {cmd, value};

    esp_err_t ret = i2c_master_transmit(bmi160_handle,buff,sizeof(buff),pdMS_TO_TICKS(100));
    if(ret != ESP_OK){
        ESP_LOGE("Transfer CMD","CMD failed %s",esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("CMD","CMD 0x%02X were transfered successfully",value);
    return ESP_OK;
}

esp_err_t BMI160_write(uint8_t reg, uint8_t data){
    uint8_t buf[2] = {reg,data};
    esp_err_t ret = i2c_master_transmit(bmi160_handle,buf,sizeof(buf),pdMS_TO_TICKS(100));
    if(ret != ESP_OK){
        ESP_LOGE("Write","Failed to write %s",esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("Write","Data 0x%02X was transfered to reg 0x%02X",data,reg);
    return ESP_OK;
}

esp_err_t BMI160_Read(uint8_t reg, uint8_t *value){
    esp_err_t ret = i2c_master_transmit_receive(bmi160_handle,&reg,1,value,1,pdMS_TO_TICKS(100)); // 15ms là đủ
    if(ret != ESP_OK){
        ESP_LOGE("Read","Data failed to read %s",esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t Data_function(uint32_t *real_time){
    uint8_t reg = BMI_DATA_ADDR;

    esp_err_t ret = i2c_master_transmit_receive(bmi160_handle,&reg,1,data_array,sizeof(data_array),pdMS_TO_TICKS(100));
    if(ret != ESP_OK){
        i2c_master_bus_reset(i2c1_bus_handle);
        return ret;
    }

    // raw data
    bmi160_data.GYRO_X = (int16_t)((data_array[1] << 8) | data_array[0]);
    bmi160_data.GYRO_Y = (int16_t)((data_array[3] << 8) | data_array[2]);
    bmi160_data.GYRO_Z = (int16_t)((data_array[5] << 8) | data_array[4]);

    bmi160_data.ACC_X = (int16_t)((data_array[7] << 8) | data_array[6]);
    bmi160_data.ACC_Y = (int16_t)((data_array[9] << 8) | data_array[8]);
    bmi160_data.ACC_Z = (int16_t)((data_array[11] << 8)| data_array[10]);
    *real_time = (uint32_t)((data_array[14] << 16) | (data_array[13] << 8) | (data_array[12]));
    return ret;
}

void BMI160_Calibate(void){
    ESP_LOGI("Calib","calib the parameters");
    float st_gx = 0, st_gy = 0, st_gz = 0;
    float st_ax = 0, st_ay = 0;
    int sample_count = 1000;
    uint32_t dummy_time;

    for (int i = 0; i < sample_count; i++)
    {
        if(Data_function(&dummy_time) == ESP_OK){
            st_gx += (float)(bmi160_data.GYRO_X /gyro_sens);
            st_gy += (float)(bmi160_data.GYRO_Y / gyro_sens);
            st_gz += (float)(bmi160_data.GYRO_Z / gyro_sens);

            st_ax += (float)(bmi160_data.ACC_X / acc_sens);
            st_ay += (float)(bmi160_data.ACC_Y / acc_sens);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gx_os = st_gx / sample_count;
    gy_os  = st_gy / sample_count;
    gz_os = st_gz / sample_count;

    ax_os = st_ax / sample_count;
    ay_os = st_ay / sample_count;
    
}

void vBMI160_Sensor_Task(void *pvParameter){
    uint32_t current_time;
    static bool first_run = true;

    while(1) {
        if(Data_function(&current_time) == ESP_OK) {
            if (first_run) {
                last_sensor_time = current_time;
                roll = atan2f(sensor_val.ay, sensor_val.az) * RAD_TO_DEG;
                pitch = atan2f(-sensor_val.ax, sqrtf(sensor_val.ay*sensor_val.ay + sensor_val.az*sensor_val.ay)) * RAD_TO_DEG;
                first_run = false;
                continue; 
            }

            // Tính toán khoảng cách thời gian 24-bit chuẩn
            uint32_t diff = (current_time >= last_sensor_time) ? 
                            (current_time - last_sensor_time) : 
                            (0xFFFFFF - last_sensor_time + current_time + 1);
            
            // 1 bit = 39.0625 micro giây. Đổi sang giây: / 1.000.000
            delta_t = (float)diff * 0.0000390625f; 

            // Giới hạn delta_t (Safety check)
            // Nếu delta_t quá lớn (do lỗi đọc), gán nó bằng mốc 10ms (0.01f)
            if (delta_t > 0.2f || delta_t <= 0) delta_t = 0.01f;

            last_sensor_time = current_time;

            sensor_val.ax = ((float)bmi160_data.ACC_X / acc_sens) - ax_os;
            sensor_val.ay = ((float)bmi160_data.ACC_Y / acc_sens) - ay_os;
            sensor_val.az = ((float)bmi160_data.ACC_Z / acc_sens);

            sensor_val.gx = ((float)bmi160_data.GYRO_X / gyro_sens) - gx_os;
            sensor_val.gy = ((float)bmi160_data.GYRO_Y / gyro_sens) - gy_os;
            sensor_val.gz = ((float)bmi160_data.GYRO_Z / gyro_sens) - gz_os;

            if(fabs(sensor_val.gz) < 0.4f){
                sensor_val.gz = 0.0f;
            }

            cal_flight_angle(&sensor_val, delta_t);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

//printf("The ax = %.1f, ay = %.1f, az = %.1f \n",sensor_val.ax,sensor_val.ay,sensor_val.az);
//printf("The gx = %.1f, gy = %.1f, gz = %.1f \n",sensor_val.gx,sensor_val.gy,sensor_val.gz);

esp_err_t BMI160_Setup(void){

    BMI160_CMD(BMI_CMD,0x6B); // soft reset
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t id;
    esp_err_t ret = BMI160_Read(CHIP_ID,&id);
    if(ret != ESP_OK){
        ESP_LOGE("ID","Failed to read the sensor id");
        return ret;
    }
    ESP_LOGI("ID","Object's ID is 0x%02X",id);

    BMI160_CMD(BMI_CMD,0x11); // ACC in normal
    vTaskDelay(pdMS_TO_TICKS(5)); // wait 3.8ms
    BMI160_CMD(BMI_CMD,0x15); // Gyro in normal
    vTaskDelay(pdMS_TO_TICKS(80)); // wait 80ms

    uint8_t state;
    ret = BMI160_Read(PMU_STATUS,&state);
    if(ret != ESP_OK){
        ESP_LOGE("PMU_STATUS","Failed to read reg %s",esp_err_to_name(ret));
        return ret;
    }

    printf("Value of acc pmu state is 0x%02X \n",state); 
    printf("ACC and GYRO are in normal mode \n");

    BIM160_write(ACC_CONF,sens.acc_cfg); 
    BIM160_write(ACC_RANGE,sens.acc_range); 
    BIM160_write(GYRO_CONF,sens.gyro_cfg); 
    BIM160_write(GYRO_RANGE,sens.gyro_range);

    return ESP_OK;
}
