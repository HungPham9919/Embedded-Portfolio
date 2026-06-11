#include "nrf24.h"

sem_t sem_tx_request;
sem_t sem_rx_complete;
pthread_t rf_thread_id;
pthread_t lan_thread_id;

pthread_t test_rx_thread_id; // receive

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
    nrf_write_reg(SETUP_AW, 0x03);   
    usleep(2000); 
    nrf_write_reg(RF_CH, 0x02);      
    usleep(2000); 
    nrf_write_reg(RF_SETUP, 0x01);   
    usleep(2000); 
    nrf_write_reg(EN_AA, 0x00);      
    usleep(2000); 
    
    // KHỞI TẠO LUỒNG NHẬN CHO 5 DRONE CÙNG LÚC TRÊN CÁC PIPE 
    nrf_write_reg(EN_RXADDR, 0x3E);  // Mở đồng loạt 5 Pipe (Pipe 1 -> Pipe 5)
    usleep(2000); 
    
    // Pipe 0 và Pipe 1 ghi đầy đủ 5 bytes địa chỉ gốc
    nrf_Write_bytes(RX_ADDR_P1, drone_addresses[0], 5); usleep(2000); // Pipe 1 nghe Drone 1
    usleep(2000);

    nrf_write_reg(RX_ADDR_P2, drone_addresses[1][0]); usleep(2000);  // Pipe 2 nghe Drone 2
    nrf_write_reg(RX_ADDR_P3, drone_addresses[2][0]); usleep(2000);  // Pipe 3 nghe Drone 3
    nrf_write_reg(RX_ADDR_P4, drone_addresses[3][0]); usleep(2000);  // Pipe 4 nghe Drone 4
    nrf_write_reg(RX_ADDR_P5, drone_addresses[4][0]); usleep(2000);  // Pipe 5 nghe Drone 5
    
    nrf_write_reg(DYNPD, 0x3E);      usleep(2000); // Kích hoạt Dynamic Payload trên cả 5 Pipe (1-5 )
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
    struct spi_ioc_transfer trans_size = { 
        .tx_buf = (unsigned long)tx_size_cmd, 
        .rx_buf = (unsigned long)rx_size_buf, 
        .len = 2, 
        .speed_hz = spi_speed, 
        .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_size);
    
    uint8_t packet_size = rx_size_buf[1];
    if (packet_size == 0 || packet_size > 32) return 0; 
    
    usleep(20); 

    uint8_t tx_payload_buf[33] = {0};
    uint8_t rx_payload_buf[33] = {0};
    tx_payload_buf[0] = R_RX_PAYLOAD;
    
    struct spi_ioc_transfer trans_payload = { 
        .tx_buf = (unsigned long)tx_payload_buf, 
        .rx_buf = (unsigned long)rx_payload_buf, 
        .len = (uint32_t)(packet_size + 1), 
        .speed_hz = spi_speed, 
        .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_payload);
    
    memcpy(pData, &rx_payload_buf[1], packet_size);
    return packet_size;
}

void nRF24_Transmit(const uint8_t *tx_addr, const uint8_t *pData, uint8_t len) {
    // 1. Chỉ nạp địa chỉ và nạp Payload, không cấu hình đè thanh ghi CONFIG
    nrf_Write_bytes(TX_ADDR, tx_addr, 5);

    static uint8_t tx_buf[34] __attribute__ ((aligned (32)));
    tx_buf[0] = W_TX_PAYLOAD;
    memcpy(&tx_buf[1], pData, len);
    struct spi_ioc_transfer trans_payload = { .tx_buf = (unsigned long)tx_buf, .len = (uint32_t)(len + 1), .speed_hz = spi_speed, .bits_per_word = 8 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_payload);

    // 2. Kích xung CE tối thiểu 15us để đẩy gói tin lên không trung
    SET_CE(1);
    usleep(15); 
    SET_CE(0);

    // 3. Đợi cho đến khi phát xong (TX_DS) hoặc lỗi (MAX_RT)
    uint32_t timeout = 0;
    uint8_t status = 0;
    while (timeout < 1000) {
        status = nrf_read_reg(STATUS);
        if (status & ((1 << 5) | (1 << 4))) { 
            break;
        }
        usleep(10);
        timeout++;
    }

    // 4. Xóa cờ trạng thái phát
    nrf_write_reg(STATUS, 0x30); 

    // Nếu bị kẹt bộ phát do tối đa lượt gửi, dọn sạch hàng đợi TX
    if (status & (1 << 4)) {
        uint8_t flush_tx_cmd = FLUSH_TX;
        struct spi_ioc_transfer trans_f = { .tx_buf = (unsigned long)&flush_tx_cmd, .len = 1, .speed_hz = spi_speed };
        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_f);
    }
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
        printf("\n[HỆ THỐNG] Dang tat Gateway an toan...\n");
        
        // 1. Hạ chân CE để ngắt chip nRF24L01 lập tức
        if (ce_request) {
            gpiod_line_request_set_value(ce_request, ce_line_num, 0);
        }

        // 2. Kích hoạt toàn bộ Semaphore để giải thoát cho các luồng đang bị treo (sem_wait / sem_timedwait)
        sem_post(&sem_tx_request);
        sem_post(&sem_rx_complete);

        // 3. Ép hủy các luồng phụ đang chạy ngầm để tránh kẹt Kernel
        pthread_cancel(lan_thread_id);
        pthread_cancel(rf_thread_id);

        // 4. Chờ một vài mili giây cho các luồng giải phóng bộ nhớ sạch sẽ
        usleep(50000);

        printf("[HỆ THỐNG] Da giai phong tai nguyen. Thoat!\n");
        exit(0); 
    }
}

void* RF_Gateway_Task(void *arg) {
    (void)arg;
    uint8_t rx_test_buffer[33]; 
    const char* target_responses[5] = {"DONE1", "DONE2", "DONE3", "DONE4", "DONE5"};

    while(1) {
        // 1. Chờ lệnh từ luồng LAN bàn phím
        printf("Change \n");
        fflush(stdout); 
        
        sem_wait(&sem_tx_request); 
        
        // --- PHA 1: CHUẨN BỊ PHÁT ---
        SET_CE(0); 
        Switch_To_TX_Mode();
        usleep(5000); // 🌟 TRỄ BẢO HIỂM 1: Chờ 5ms để mạch PLL của chip ổn định hoàn toàn ở chế độ TX

        // Xả sạch bộ đệm phát cũ và dọn sạch cờ ngắt
        nrf_write_reg(FLUSH_TX, 0xE1); 
        nrf_write_reg(STATUS, 0x70);   
        usleep(100);

        // Gọt sạch khoảng trắng ở cuối lệnh
        int len_to_send = strlen(global_cmd);
        while (len_to_send > 0 && (global_cmd[len_to_send - 1] == ' '  || 
                                   global_cmd[len_to_send - 1] == '\r' || 
                                   global_cmd[len_to_send - 1] == '\n')) {
            global_cmd[len_to_send - 1] = '\0';
            len_to_send--;
        }

        if(len_to_send > 0 && len_to_send <= 32) {
            printf("[GATEWAY_TX] Đang phát lệnh thực tế: '%s' (Độ dài: %d byte)...\n", global_cmd, len_to_send);
            fflush(stdout);
            
            // Ép địa chỉ Pipe 0 trùng địa chỉ đích để nhận gói Auto-ACK ngầm
            nrf_Write_bytes(0x0A, target_drone_addr, 5); 

            // Gọi hàm gốc phát sóng (Hàm này tự giật CE, tự đợi cờ xong tự dọn cờ STATUS)
            nRF24_Transmit(target_drone_addr, (uint8_t*)global_cmd, len_to_send);
        }
        
        // --- PHA 2: CHUYỂN TIẾP VẬT LÝ ---
        SET_CE(0);    // Đảm bảo hạ chân CE xuống thấp hoàn toàn
        usleep(5000); // 🌟 TRỄ BẢO HIỂM 2: Ép hoãn 5ms để chip nRF hoàn tất việc phát/nhận ACK vật lý, không lật CONFIG quá đột ngột

        // --- PHA 3: CHUYỂN SANG THU PHẢN HỒI ---
        Switch_To_RX_Mode();       
        nrf_write_reg(STATUS, 0x70);    // Dọn sạch cờ ngắt phát vừa xong
        nrf_write_reg(FLUSH_RX, 0xE2);  // Xả sạch bộ đệm nhận để đón dữ liệu mới sạch sẽ
        usleep(100);
        
        SET_CE(1); // Dựng tai nghe sóng lên liên tục
        printf("[GATEWAY] Dang treo luong doi cac chuoi DONE(1-5) tu Drone...\n");
        fflush(stdout);
        
        int rx_timeout_counter = 0;
        int has_response = 0;

        while (1) { 
            uint8_t status = nrf_read_reg(STATUS);

            // Nếu Bit 6 (RX_DR) nổ -> Có gói tin phản hồi từ Drone về
            if (status & (1 << 6)) { 
                SET_CE(0); // Hạ CE để đọc dữ liệu qua SPI ổn định

                uint8_t len = nRF24_Read_Buffer(rx_test_buffer); 
                if (len > 0 && len <= 32) {
                    rx_test_buffer[len] = '\0'; 
                    
                    printf("\n[GATEWAY_POLLING -> NHẬN ĐƯỢC] Data: '%s' | Độ dài: %d byte | STATUS: 0x%02X\n", 
                           rx_test_buffer, len, status);
                    fflush(stdout);

                    for (int i = 0; i < 5; i++) {
                        if (strcmp((char*)rx_test_buffer, target_responses[i]) == 0) {
                            has_response = 1;
                            printf("[GATEWAY] Khop chinh xac voi mang kiem tra: %s\n", target_responses[i]);
                            fflush(stdout);
                            break;
                        }
                    }

                    if (has_response) {
                        sem_post(&sem_rx_complete);  
                        nrf_write_reg(STATUS, 0x40); // Xóa cờ RX_DR
                        break;                       
                    }
                }
                
                nrf_write_reg(STATUS, 0x40); 
                SET_CE(1); // Tiếp tục nghe tiếp nếu là gói nhiễu
            }

            usleep(2000); // Giãn cách 2ms giảm tải CPU cho Pi

            // Giữ thời gian nghe sóng lâu hơn một chút (250 chu kỳ * 2ms = 500ms) để đợi STM32 xử lý lệnh
            if (++rx_timeout_counter > 250) {
                printf("[GATEWAY -> TIMEOUT RX] Đã quá 500ms không nhận được gói DONE nào. Tự động thoát chu kỳ.\n");
                fflush(stdout);
                break;
            }
        }
        
        SET_CE(0); // Hạ chân CE về trạng thái an toàn
    }
    return NULL;
}

void* LAN_Network_Task(void* arg) {
    (void)arg; 
    char input_buf[100];

    printf("\n=========================================================\n");
    printf("[LAN_TASK] CHẾ ĐỘ NHẬP LỆNH BÀN PHÍM - CỐ ĐỊNH DRONE 1\n");
    printf("=========================================================\n");

    while(1) {
        // 1. Ép cứng địa chỉ đích là Drone 1
        memcpy(target_drone_addr, drone_addresses[0], 5);

        // 2. Chờ người dùng nhập nội dung lệnh từ bàn phím
        printf("\n[LAN_TASK] Nhập lệnh gửi DRONE 1: ");
        fflush(stdout);
        
        // Đọc chuỗi ký tự từ bàn phím
        if (fgets(input_buf, sizeof(input_buf), stdin) == NULL) {
            continue;
        }

        // Xóa ký tự xuống dòng '\n' do hàm fgets tự động nhận khi ấn Enter
        input_buf[strcspn(input_buf, "\n")] = '\0';

        // Nếu chỉ ấn Enter mà không gõ gì thì bỏ qua, bắt nhập lại
        if (strlen(input_buf) == 0) {
            printf("[LỖI] Lệnh không được để trống!\n");
            continue;
        }

        // Copy lệnh vừa nhập vào biến toàn cục global_cmd để luồng RF bốc đi phát
        strncpy(global_cmd, input_buf, sizeof(global_cmd) - 1);
        global_cmd[sizeof(global_cmd) - 1] = '\0'; // Bảo hiểm kết thúc chuỗi

        printf("\n---------------------------------------------------------");
        printf("\n[LAN_TASK] Đã nhận lệnh bàn phím: '%s' -> Đang gửi cho DRONE 1 (0x%02X)\n", 
                global_cmd, target_drone_addr[0]);

        // 3. Đánh thức luồng RF_Gateway_Task dậy để đẩy sóng đi
        printf("[LAN_TASK] Bắn tín hiệu (sem_post), kích hoạt RF_TASK thực thi...\n");
        sem_post(&sem_tx_request);

        // 4. Treo luồng chờ phản hồi ngược lại từ Drone 1 (Timeout tối đa 3 giây)
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 3; 

        printf("[LAN_TASK] Đang treo luồng chờ xác nhận 'DONE' từ Drone 1...\n");
        if (sem_timedwait(&sem_rx_complete, &ts) == 0) {
            printf("[LAN_TASK -> SUCCESS] Quá ngon! Nhận được xác nhận 'DONE' từ Drone 1.\n");
        } else {
            printf("[LAN_TASK -> TIMEOUT] Quá hạn 3 giây! Không thấy Drone 1 báo cáo.\n");
        }
        printf("---------------------------------------------------------\n");
    }
    return NULL;
}

void* Test_Pi_Receive_Only_Task(void *arg) {
    (void)arg;
    uint8_t rx_test_buffer[33];
    printf("\n=========================================================\n");
    printf("[TEST_RX] KICH HOAT CHE DO CHI DOI DOC DATA LIEN TUC...\n");
    printf("[TEST_RX] Dang lang nghe Drone tai dia chi: 0x%02X\n", drone_addresses[4][0]); 
    printf("=========================================================\n");

    // 1. Cấu hình ban đầu an toàn
    SET_CE(0);
    uint8_t config = nrf_read_reg(CONFIG);
    config |= (1 << 1) | (1 << 0); // PWR_UP, PRIM_RX
    nrf_write_reg(CONFIG, config);
    
    nrf_write_reg(STATUS, 0x70);    // Xóa sạch cờ cũ
    nrf_write_reg(FLUSH_RX, 0xE2);   // Xả sạch FIFO
    SET_CE(1);                      // Bắt đầu mở tai nghe sóng

    while(1) {
        // Đợi sự kiện cạnh xuống từ chân IRQ (Timeout 500ms)
        int ret = gpiod_line_request_wait_edge_events(irq_request, 500000000LL); 
        
        if (ret > 0) {
            // Đọc và giải phóng hàng đợi sự kiện của Linux gpiod
            gpiod_line_request_read_edge_events(irq_request, event_buffer, 1);

            uint8_t status = nrf_read_reg(STATUS);
            
            // Kiểm tra xem có đúng ngắt nhận dữ liệu (RX_DR) không
            if (status & (1 << 6)) { 
                SET_CE(0); // Hạ CE xuống để đọc SPI an toàn

                uint8_t len = nRF24_Read_Buffer(rx_test_buffer);
                if (len > 0 && len <= 32) {
                    rx_test_buffer[len] = '\0'; 
                    printf("[TEST_RX -> NHAN DUOC] Data: '%s' | Do dai: %d byte | STATUS: 0x%02X | From Pipe: %d\n",
                           rx_test_buffer, len, status, (status >> 1) & 0x07);
                    fflush(stdout);
                } else {
                    // 🌟 THIẾU SÓT BỔ SUNG: Nếu đọc độ dài ra 0 hoặc rác, ép xả khẩn cấp để cứu FIFO
                    nrf_write_reg(FLUSH_RX, 0xE2);
                }

                nrf_write_reg(STATUS, 0x40);   // Viết 1 vào bit 6 để clear ngắt RX_DR
                nrf_write_reg(FLUSH_RX, 0xE2); // Bảo hiểm dọn sạch bộ đệm
                SET_CE(1);                     // Tiếp tục nghe sóng
            } else {
                // Nếu ngắt nổ ra do nguyên nhân khác (TX_DS, MAX_RT), vẫn phải dọn thanh ghi trạng thái
                nrf_write_reg(STATUS, 0x70);
            }
        }
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

    // Kích hoạt hệ điều hành Linux chạy đa luồng song song

    pthread_create(&rf_thread_id, NULL, RF_Gateway_Task, NULL);
    pthread_create(&lan_thread_id, NULL, LAN_Network_Task, NULL);

    // Neo giữ luồng chính không giải phóng bộ nhớ hệ thống

    pthread_join(rf_thread_id, NULL); // Transmit
    pthread_join(lan_thread_id, NULL);


    // Kích hoạt DUY NHẤT luồng chỉ nhận dữ liệu liên tục
    // pthread_create(&test_rx_thread_id, NULL, Test_Pi_Receive_Only_Task, NULL);
    // pthread_join(test_rx_thread_id, NULL);

    return 0;
}
