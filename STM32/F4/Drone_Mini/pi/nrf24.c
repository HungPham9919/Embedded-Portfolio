#include "nrf24.h"

sem_t sem_tx_request;
sem_t sem_rx_complete;

int spi_fd = -1;
struct gpiod_chip *gpio_chip = NULL;
struct gpiod_line_request *ce_request = NULL;
struct gpiod_line_request *irq_request = NULL; 
struct gpiod_edge_event_buffer *event_buffer;

unsigned int ce_line_num = 25;   // Chân 22 trên board
unsigned int irq_line_num = 23;  // Chân 16 trên board (GPIO23)
char global_cmd[32];
uint8_t rx_buffer[32];

uint32_t spi_speed = 1000000;
uint8_t target_drone_addr[5] = {0x01, 0xCC, 0xCC, 0xCC, 0xCC};
static const uint8_t drone_addresses[5][5] = {
    {0x01, 0xCC, 0xCC, 0xCC, 0xCC}, // Drone 1 - pipe 1
    {0x02, 0xCC, 0xCC, 0xCC, 0xCC}, // Drone 2 - pipe 2
    {0x03, 0xCC, 0xCC, 0xCC, 0xCC}, // Drone 3 - pipe 3
    {0x04, 0xCC, 0xCC, 0xCC, 0xCC}, // Drone 4 - pipe 4
    {0x05, 0xCC, 0xCC, 0xCC, 0xCC}  // Drone 5 - pipe 5
};

void SET_CE(int state) {
    if (ce_request) {
        gpiod_line_request_set_value(ce_request, ce_line_num, state ? 1 : 0);
        usleep(15);            
    }
}

void nrf_write_reg(uint8_t reg, uint8_t value){
    uint8_t buff[2] = { (W_REGISTER | reg), value };
    struct spi_ioc_transfer trans = { .tx_buf = (unsigned long)buff, .len = 2, .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans);
}

uint8_t nrf_read_reg(uint8_t reg){
    uint8_t tx_buf[2] = { (R_REGISTER | reg), 0xFF };
    uint8_t rx_buf[2] = { 0 };
    struct spi_ioc_transfer trans = { .tx_buf = (unsigned long)tx_buf, .rx_buf = (unsigned long)rx_buf, .len = 2, .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans);
    return rx_buf[1];
}

void nrf_Write_bytes(uint8_t reg, const uint8_t *byte, uint8_t len){
    static uint8_t tx_buf[34] __attribute__ ((aligned (32)));
    tx_buf[0] = (W_REGISTER | reg);
    memcpy(&tx_buf[1], byte, len);
    struct spi_ioc_transfer trans = { .tx_buf = (unsigned long)tx_buf, .len = (uint32_t)(len + 1), .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans);
}

void SPI_nRF24_Config_RX(const uint8_t *rx_addr){
    (void)rx_addr;
    SET_CE(0);          
    usleep(20000);      

    nrf_write_reg(CONFIG, 0x00); 
    usleep(10000);

    nrf_write_reg(SETUP_AW, 0x03);   usleep(2000); 
    nrf_write_reg(RF_CH, 0x02);      usleep(2000); 
    nrf_write_reg(RF_SETUP, 0x01);   usleep(2000); 
    
    nrf_write_reg(EN_AA, 0x00);      usleep(2000); 
    
    // 🌟 KHỞI TẠO LUỒNG NHẬN CHO 5 DRONE CÙNG LÚC TRÊN CÁC PIPE CHÊNH LỆCH
    nrf_write_reg(EN_RXADDR, 0x3E);  usleep(2000); // Mở đồng loạt 5 Pipe (Pipe 1 -> Pipe 5)
    
    // Pipe 0 và Pipe 1 ghi đầy đủ 5 bytes địa chỉ gốc
    nrf_Write_bytes(RX_ADDR_P1, drone_addresses[0], 5); usleep(2000); // Pipe 1 nghe Drone 1
    usleep(2000);
    // Pipe 2, 3, 4 chia sẻ chung 4 bytes cuối của Pipe 1, chỉ cần cấu hình byte Prefix đầu tiên
    nrf_write_reg(RX_ADDR_P2, drone_addresses[1][0]); usleep(2000);  // Pipe 2 nghe Drone 2
    nrf_write_reg(RX_ADDR_P3, drone_addresses[2][0]); usleep(2000);  // Pipe 3 nghe Drone 3
    nrf_write_reg(RX_ADDR_P4, drone_addresses[3][0]); usleep(2000);  // Pipe 4 nghe Drone 4
    nrf_write_reg(RX_ADDR_P5, drone_addresses[4][0]); usleep(2000);  // Pipe 5 nghe Drone 5
    
    nrf_write_reg(DYNPD, 0x1F);      usleep(2000); // Kích hoạt Dynamic Payload trên cả 5 Pipe
    nrf_write_reg(FEATURE, 0x04);    usleep(2000); 

    nrf_write_reg(STATUS, 0x70);     usleep(2000); 
    
    uint8_t cmd_flush = FLUSH_RX;
    struct spi_ioc_transfer trans_flush = { .tx_buf = (unsigned long)&cmd_flush, .len = 1, .speed_hz = spi_speed };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_flush);
    usleep(2000);

    nrf_write_reg(CONFIG, 0x0F);     
    usleep(20000);                    
    
    SET_CE(1);                       
    usleep(1000); 
}

uint8_t nRF24_Read_Buffer(uint8_t *pData) {
    uint8_t tx_size_cmd[2] = { PAYLOAD_WIDTH, 0xFF };
    uint8_t rx_size_buf[2] = { 0 };
    struct spi_ioc_transfer trans_size = { .tx_buf = (unsigned long)tx_size_cmd, .rx_buf = (unsigned long)rx_size_buf, .len = 2, .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_size);
    
    uint8_t packet_size = rx_size_buf[1];
    if (packet_size == 0 || packet_size > 32) return 0; 
    
    usleep(20); 

    uint8_t tx_payload_buf[33] = {0};
    uint8_t rx_payload_buf[33] = {0};
    tx_payload_buf[0] = R_RX_PAYLOAD;
    
    struct spi_ioc_transfer trans_payload = { .tx_buf = (unsigned long)tx_payload_buf, .rx_buf = (unsigned long)rx_payload_buf, .len = (uint32_t)(packet_size + 1), .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_payload);
    
    memcpy(pData, &rx_payload_buf[1], packet_size);
    return packet_size;
}

void nRF24_Transmit(const uint8_t *tx_addr, const uint8_t *pData, uint8_t len) {
    SET_CE(0); 
    uint8_t cmd_flush = FLUSH_TX;
    struct spi_ioc_transfer trans_flush = { .tx_buf = (unsigned long)&cmd_flush, .len = 1, .speed_hz = spi_speed };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_flush);

    nrf_Write_bytes(TX_ADDR, tx_addr, 5);

    nrf_write_reg(CONFIG, 0x0E); 
    usleep(150); 

    static uint8_t tx_buf[34] __attribute__ ((aligned (32)));
    tx_buf[0] = W_TX_PAYLOAD;
    memcpy(&tx_buf[1], pData, len);
    struct spi_ioc_transfer trans_payload = { .tx_buf = (unsigned long)tx_buf, .len = (uint32_t)(len + 1), .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_payload);

    SET_CE(1);
    usleep(15); 
    SET_CE(0);

    uint32_t timeout = 0;
    uint8_t status = 0;
    while (timeout < 1000) {
        status = nrf_read_reg(STATUS);
        if (status & ((1 << 5) | (1 << 4))) { 
            break;
        }
        usleep(50);
        timeout++;
    }

    nrf_write_reg(STATUS, 0x30); 

    if (status & (1 << 4)) {
        uint8_t flush_tx_cmd = FLUSH_TX;
        struct spi_ioc_transfer trans_f = { .tx_buf = (unsigned long)&flush_tx_cmd, .len = 1, .speed_hz = spi_speed };
        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_f);
    }
    
    nrf_write_reg(CONFIG, 0x0F);                     
    usleep(150);                                     
    SET_CE(1);                                       
}

void Switch_To_TX_Mode(void) {
    SET_CE(0);
    uint8_t config = nrf_read_reg(CONFIG);
    config &= ~(1 << 0); 
    nrf_write_reg(CONFIG, config);
    nrf_write_reg(STATUS, 0x70); 
    
    uint8_t flush_cmd = FLUSH_TX;
    struct spi_ioc_transfer trans_flush = { .tx_buf = (unsigned long)&flush_cmd, .len = 1, .speed_hz = spi_speed };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_flush);
}

void Switch_To_RX_Mode(void) {
    SET_CE(0);
    uint8_t config = nrf_read_reg(CONFIG);
    config |= (1 << 0); 
    nrf_write_reg(CONFIG, config);
    nrf_write_reg(STATUS, 0x70);
    
    uint8_t flush_cmd = FLUSH_RX;
    struct spi_ioc_transfer trans_flush = { .tx_buf = (unsigned long)&flush_cmd, .len = 1, .speed_hz = spi_speed };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_flush);
    SET_CE(1);
}

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("\n[HỆ THỐNG] Đang tắt Gateway an toàn...\n");
        SET_CE(0); 
        if (ce_request) gpiod_line_request_release(ce_request);
        if (irq_request) gpiod_line_request_release(irq_request);
        if (event_buffer) gpiod_edge_event_buffer_free(event_buffer);
        if (gpio_chip) gpiod_chip_close(gpio_chip);
        if (spi_fd >= 0) close(spi_fd);
        sem_destroy(&sem_tx_request);
        sem_destroy(&sem_rx_complete);
        exit(0);
    }
}

void* RF_Gateway_Task(void *arg) {
    (void)arg;
    while(1) {
        // Luồng ngủ hoàn toàn cho đến khi có yêu cầu phát từ mạng LAN
        sem_wait(&sem_tx_request); 

        printf("[RF_TASK] Thức giấc! Gửi lệnh: '%s' tới địa chỉ: 0x%02X...\n", global_cmd, target_drone_addr[0]);
        Switch_To_TX_Mode();
        nRF24_Transmit(target_drone_addr, (uint8_t*)global_cmd, strlen(global_cmd));
        
        // Chờ chip nRF24L01 xử lý xong lượt truyền (Phát xong hoặc chạm đỉnh Retransmit)
        while( !(nrf_read_reg(STATUS) & ((1 << 5) | (1 << 4))) ); 
        nrf_write_reg(STATUS, 0x30); // Xóa cờ TX_DS / MAX_RT

        printf("[RF_TASK] Đang mở RX Mode chờ Drone phản hồi trong 100ms...\n");
        Switch_To_RX_Mode();

        // Chờ sự kiện ngắt vật lý từ chân IRQ (Cấu hình Timeout 500ms)
        int ret = gpiod_line_request_wait_edge_events(irq_request, 500000000LL); 
        
        if (ret > 0) { 
            gpiod_line_request_read_edge_events(irq_request, event_buffer, 1); 
            uint8_t status = nrf_read_reg(STATUS);

            if (status & (1 << 6)) { // Cờ RX_DR báo nhận dữ liệu thành công
                SET_CE(0);
                uint8_t len = nRF24_Read_Buffer(rx_buffer);
                if (len > 0) {
                    rx_buffer[len] = '\0';
                    
                    // Đọc thanh ghi để biết chính xác Drone từ Pipe nào vừa bắn về
                    uint8_t pipe_id = (status >> 1) & 0x07; 
                    printf("[RF_TASK -> OK] Nhận từ Drone Pipe [%d]: %s\n", pipe_id, rx_buffer);
                    
                    if (strcmp((char*)rx_buffer, "DONE") == 0) {
                        sem_post(&sem_rx_complete); // Phát tín hiệu báo thành công cho luồng LAN
                    }
                }
                nrf_write_reg(STATUS, 0x40); 
                SET_CE(1);
            }
        } 
        else { 
            printf("[RF_TASK -> WARN] Hết hạn 100ms! Không nhận được phản hồi 'DONE'.\n");
        }
    }
    return NULL;
}

// =================================================================
// TASK 2: LUỒNG ĐỒNG BỘ LAN VÀ PHÂN CHIA LỆNH CHO 5 DRONE
// =================================================================
// void* LAN_Network_Task(void *arg) {
//     (void)arg;
//     uint8_t current_drone_index = 0;

//     while(1) {
//         // Giả lập: Cứ 4 giây hệ thống điều khiển bầy đàn quét sang ra lệnh cho con tiếp theo
//         sleep(4); 

//         // 🌟 GIẢI QUYẾT BÀI TOÁN 5 DRONE: Ánh xạ luân phiên địa chỉ của từng con
//         memcpy(target_drone_addr, drone_addresses[current_drone_index], 5);
//         strcpy(global_cmd, "TAKEOFF"); 

//         printf("\n[LAN_TASK] Tạo yêu cầu điều khiển -> DRONE %d (Prefix: 0x%02X)\n", 
//                 current_drone_index + 1, target_drone_addr[0]);

//         // Đánh thức luồng RF thực thi nhiệm vụ
//         sem_post(&sem_tx_request);

//         struct timespec ts;
//         clock_gettime(CLOCK_REALTIME, &ts);
//         ts.tv_sec += 1; // Đợi tối đa 1 giây cho Drone phản hồi thực thi xong

//         if (sem_timedwait(&sem_rx_complete, &ts) == 0) {
//             printf("[LAN_TASK] Thành công! DRONE %d đã báo cáo hoàn thành tác vụ.\n", current_drone_index + 1);
//         } else {
//             printf("[LAN_TASK -> ERR] Thất bại! DRONE %d không hoàn tất tác vụ (Timeout).\n", current_drone_index + 1);
//         }

//         // Chuyển sang con Drone kế tiếp trong danh sách (1 -> 2 -> 3 -> 4 -> 5 -> Quay lại 1)
//         current_drone_index = (current_drone_index + 1) % 5;
//     }
//     return NULL;
// }

// =================================================================
// TASK 2: LUỒNG GIẢ LẬP MẠNG LAN - KHÓA CỨNG ĐIỀU KHIỂN DRONE 1
// =================================================================

void* LAN_Network_Task(void* arg) {
    (void)arg; 

    printf("\n[LAN_TASK] Đã kích hoạt chế độ Giả lập lệnh LAN cho Drone 1...\n");
    int command_toggle = 0;
    while(1) {
        // Cứ mỗi 5 giây, giả lập có một lệnh "TAKEOFF" từ mạng LAN nạp xuống Pi 5
        sleep(5); 
        memcpy(target_drone_addr, drone_addresses[0], 5);

        // 2. Nạp nội dung lệnh xen kẽ dựa vào biến command_toggle
        if (command_toggle == 0) {
            strcpy(global_cmd, "STATE"); 
            command_toggle = 1; // Chu kỳ sau lật sang lệnh tiếp theo
        } else {
            strcpy(global_cmd, "FLY"); 
            command_toggle = 0;
        }

        printf("\n---------------------------------------------------------");
        printf("\n[LAN_TASK] Giả lập nhận lệnh mạng: '%s' -> Gửi cho DRONE 1 (0x%02X)\n", 
                global_cmd, target_drone_addr[0]);

        // 3. Đánh thức luồng RF_Gateway_Task dậy để đẩy sóng đi
        printf("[LAN_TASK] Bắn tín hiệu (sem_post), kích hoạt RF_TASK thực thi...\n");
        sem_post(&sem_tx_request);

        // 4. Đứng đợi phản hồi ngược lại từ Drone 1 (Timeout tối đa 1 giây)
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 3; 

        printf("[LAN_TASK] Đang treo luồng chờ xác nhận 'DONE' từ Drone 1...\n");
        if (sem_timedwait(&sem_rx_complete, &ts) == 0) {
            printf("[LAN_TASK -> SUCCESS] Quá ngon! Nhận được xác nhận 'DONE' từ Drone 1.\n");
        } else {
            printf("[LAN_TASK -> TIMEOUT] Quá hạn 2 giây! Không thấy Drone 1 báo cáo.\n");
        }
        printf("---------------------------------------------------------\n");
    }
    return NULL;
}

// =================================================================
// HÀM KHỞI TẠO PHẦN CỨNG TOÀN CỤC
// =================================================================
int Initialized(void){
    signal(SIGINT, sig_handler);
    printf("[HỆ THỐNG] Đang khởi tạo GPIO (CE & IRQ Edge) bằng libgpiod V2...\n");
    
    gpio_chip = gpiod_chip_open("/dev/gpiochip4");
    if (!gpio_chip) gpio_chip = gpiod_chip_open("/dev/gpiochip0");
    if (!gpio_chip) { perror("Lỗi mở gpiochip"); return -1; }

    struct gpiod_line_settings *ce_settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(ce_settings, GPIOD_LINE_DIRECTION_OUTPUT);
    struct gpiod_line_config *ce_line_cfg = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(ce_line_cfg, &ce_line_num, 1, ce_settings);
    struct gpiod_request_config *ce_req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(ce_req_cfg, "nRF24_CE");
    ce_request = gpiod_chip_request_lines(gpio_chip, ce_req_cfg, ce_line_cfg);
    gpiod_line_settings_free(ce_settings); gpiod_line_config_free(ce_line_cfg); gpiod_request_config_free(ce_req_cfg);

    if (!ce_request) { perror("Lỗi cấu hình chân CE"); gpiod_chip_close(gpio_chip); return -1; }

    struct gpiod_line_settings *irq_settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(irq_settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(irq_settings, GPIOD_LINE_EDGE_FALLING); 
    gpiod_line_settings_set_bias(irq_settings, GPIOD_LINE_BIAS_PULL_UP);          
    
    struct gpiod_line_config *irq_line_cfg = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(irq_line_cfg, &irq_line_num, 1, irq_settings);
    struct gpiod_request_config *irq_req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(irq_req_cfg, "nRF24_IRQ_EDGE");
    irq_request = gpiod_chip_request_lines(gpio_chip, irq_req_cfg, irq_line_cfg);
    gpiod_line_settings_free(irq_settings); gpiod_line_config_free(irq_line_cfg); gpiod_request_config_free(irq_req_cfg);

    if (!irq_request) { perror("Lỗi cấu hình chân IRQ Edge"); gpiod_line_request_release(ce_request); gpiod_chip_close(gpio_chip); return -1; }

    // Khởi tạo bộ đệm sự kiện ngắt
    event_buffer = gpiod_edge_event_buffer_new(1);

    printf("[HỆ THỐNG] Đang mở cổng SPI Kernel với Hardware CSN...\n");
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) { perror("Lỗi mở SPI"); return -1; }

    uint8_t mode = SPI_MODE_0; 
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) { perror("Lỗi SPI Mode"); return -1; }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) { perror("Lỗi SPI Speed"); return -1; }
    
    // Mở cấu hình RX ban đầu lắng nghe mảng Drone
    SPI_nRF24_Config_RX(drone_addresses[0]);

    printf("\n=== TOÀN BỘ CẤU HÌNH PHẦN CỨNG NRF24L01 (NGẮT) ===\n");
    printf("CONFIG    : 0x%02X\n", nrf_read_reg(CONFIG));
    printf("STATUS    : 0x%02X\n", nrf_read_reg(STATUS));
    printf("FIFO_STATUS: 0x%02X\n", nrf_read_reg(FIFO_STATUS));
    printf("==================================================\n");
    return 0;
}


int main(void) {
    if (Initialized() < 0) {
        printf("[LỖI] Khởi tạo phần cứng thất bại!\n");
        return -1;
    }

    // Khởi tạo các Semaphore đồng bộ luồng dữ liệu
    sem_init(&sem_tx_request, 0, 0);
    sem_init(&sem_rx_complete, 0, 0);

    pthread_t rf_thread_id;
    pthread_t lan_thread_id;

    // Kích hoạt hệ điều hành Linux chạy đa luồng song song
    pthread_create(&rf_thread_id, NULL, RF_Gateway_Task, NULL);
    pthread_create(&lan_thread_id, NULL, LAN_Network_Task, NULL);

    // Neo giữ luồng chính không giải phóng bộ nhớ hệ thống
    pthread_join(rf_thread_id, NULL);
    pthread_join(lan_thread_id, NULL);

    return 0;
}
