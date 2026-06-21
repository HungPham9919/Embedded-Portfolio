#include "nrf24.h"
#include <zmq.h>

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
    nrf_write_reg(EN_RXADDR, 0x3F);  // Mở đồng loạt 6 Pipe  0 (Pipe 1 -> Pipe 5)
    usleep(2000); 
    
    // Pipe 0 và Pipe 1 ghi đầy đủ 5 bytes địa chỉ gốc
    nrf_Write_bytes(RX_ADDR_P1, drone_addresses[0], 5); usleep(2000); // Pipe 1 nghe Drone 1
    usleep(2000);

    nrf_write_reg(RX_ADDR_P2, drone_addresses[1][0]); usleep(2000);  // Pipe 2 nghe Drone 2
    nrf_write_reg(RX_ADDR_P3, drone_addresses[2][0]); usleep(2000);  // Pipe 3 nghe Drone 3
    nrf_write_reg(RX_ADDR_P4, drone_addresses[3][0]); usleep(2000);  // Pipe 4 nghe Drone 4
    nrf_write_reg(RX_ADDR_P5, drone_addresses[4][0]); usleep(2000);  // Pipe 5 nghe Drone 5
    
    nrf_write_reg(DYNPD, 0x3F);      usleep(2000); // Kích hoạt Dynamic Payload trên cả 6 Pipe (0-5)
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
    struct spi_ioc_transfer trans_payload = { 
        .tx_buf = (unsigned long)tx_buf, 
        .len = (uint32_t)(len + 1), 
        .speed_hz = spi_speed, 
        .bits_per_word = 8 };
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
    nrf_write_reg(STATUS, 0x70); 

    // Nếu bị kẹt bộ phát do tối đa lượt gửi, dọn sạch hàng đợi TX
    if (status & (1 << 4)) {
        uint8_t flush_tx_cmd = FLUSH_TX;
        struct spi_ioc_transfer trans_f = { 
            .tx_buf = (unsigned long)&flush_tx_cmd, 
            .len = 1, 
            .speed_hz = spi_speed };
        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_f);
    }
}

void Switch_To_TX_Mode(void) {
    SET_CE(0);
    nrf_write_reg(CONFIG, 0x0E); 
    nrf_write_reg(STATUS, 0x70); 
    
    uint8_t flush_cmd = FLUSH_TX;
    struct spi_ioc_transfer trans_flush = { 
        .tx_buf = (unsigned long)&flush_cmd, 
        .len = 1, 
        .speed_hz = spi_speed };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_flush);
}

void Switch_To_RX_Mode(void) {
    SET_CE(0); // Hạ CE để lật cấu hình an toàn
    nrf_write_reg(CONFIG, 0x0F); 
    
    // 2. Dọn dẹp trận địa: Xóa cờ ngắt và xả sạch bộ đệm
    nrf_write_reg(STATUS, 0x70);
    
    uint8_t flush_cmd = FLUSH_RX;
    struct spi_ioc_transfer trans_flush = { 
        .tx_buf = (unsigned long)&flush_cmd, 
        .len = 1, 
        .speed_hz = spi_speed };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_flush);
    
    SET_CE(1); 
    usleep(130); 
}

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("\n[HỆ THỐNG] Phát hiện Ctrl+C! Đang hủy hệ thống khẩn cấp nhưng an toàn...\n");
        
        // 1. Hạ chân CE lập tức để cô lập chip nRF24L01, ngừng thu/phát vô tuyến
        if (ce_request) {
            gpiod_line_request_set_value(ce_request, ce_line_num, 0);
        }
        usleep(1000); // Trễ 1ms để chip nhận biết trạng thái Standby-I

        // 2. Ép chip nRF24L01 về trạng thái ngủ sâu (Power Down) và xả sạch FIFO rác
        if (spi_fd >= 0) {
            // Ghi CONFIG = 0x00 (Tắt PWR_UP, tắt PRIM_RX) -> Đưa chip về trạng thái ngủ tuyệt đối
            uint8_t config_shutdown[2] = { (W_REGISTER | CONFIG), 0x00 };
            struct spi_ioc_transfer trans_cfg = { 
                .tx_buf = (unsigned long)config_shutdown, 
                .len = 2, 
                .speed_hz = spi_speed, 
                .bits_per_word = 8 
            };
            ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_cfg);

            // Xóa sạch toàn bộ cờ ngắt trên thanh ghi STATUS
            uint8_t status_clear[2] = { (W_REGISTER | STATUS), 0x70 };
            struct spi_ioc_transfer trans_stat = { 
                .tx_buf = (unsigned long)status_clear, 
                .len = 2, 
                .speed_hz = spi_speed, 
                .bits_per_word = 8 
            };
            ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_stat);

            // Xả sạch bộ đệm phát (FLUSH_TX)
            uint8_t flush_tx_cmd = FLUSH_TX;
            struct spi_ioc_transfer trans_ftx = { .tx_buf = (unsigned long)&flush_tx_cmd, .len = 1, .speed_hz = spi_speed };
            ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_ftx);

            // Xả sạch bộ đệm nhận (FLUSH_RX)
            uint8_t flush_rx_cmd = FLUSH_RX;
            struct spi_ioc_transfer trans_frx = { .tx_buf = (unsigned long)&flush_rx_cmd, .len = 1, .speed_hz = spi_speed };
            ioctl(spi_fd, SPI_IOC_MESSAGE(1), &trans_frx);
        }

        // 3. Giải thoát cho các Semaphore đang bị kẹt (nếu có luồng nào đang đợi)
        sem_post(&sem_tx_request);
        sem_post(&sem_rx_complete);

        // 4. Bấy giờ chip đã ngủ an toàn -> Ép hủy các luồng để thoát ngay lập tức, không chờ đợi
        printf("[HỆ THỐNG] Đang giải phóng các luồng dữ liệu...\n");
        pthread_cancel(rf_thread_id);
        pthread_cancel(lan_thread_id);

        // Trễ một chút cho OS thu hồi luồng
        usleep(10000);

        printf("[HỆ THỐNG] nRF24L01 đã ngủ sâu (Power Down). Thoát hoàn toàn!\n");
        exit(0); 
    }
}

void* RF_Gateway_Task(void *arg) {
    (void)arg;
    uint8_t rx_test_buffer[33]; 
    const char* target_responses[9] = {"DONE1", "DONE2", "DONE3", "DONE4", "DONE5","DONE6","DONE7","DONE8","DONE9"};

    while(1) {
        fflush(stdout); 
        
        sem_wait(&sem_tx_request); 
        
        // --- PHA 1: CHUẨN BỊ PHÁT ---
        SET_CE(0); 
        Switch_To_TX_Mode();
        usleep(5000);

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
        
            // Gọi hàm gốc phát sóng (Hàm này tự giật CE, tự đợi cờ xong tự dọn cờ STATUS)
            nRF24_Transmit(target_drone_addr, (uint8_t*)global_cmd, len_to_send);
        }
        
        // --- PHA 2: CHUYỂN TIẾP VẬT LÝ ---
        SET_CE(0);
        usleep(5000); 

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

            if (status & (1 << 6)) { 
                SET_CE(0); // Hạ CE để đọc dữ liệu qua SPI ổn định

                uint8_t len = nRF24_Read_Buffer(rx_test_buffer); 
                if (len > 0 && len <= 32) {
                    rx_test_buffer[len] = '\0'; 
                    
                    printf("\n[GATEWAY_POLLING -> NHẬN ĐƯỢC] Data: '%s' | Độ dài: %d byte | STATUS: 0x%02X\n", 
                           rx_test_buffer, len, status);
                    fflush(stdout);

                    for (int i = 0; i < 9; i++) {
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
            if (++rx_timeout_counter > 200) {
                printf("[GATEWAY -> TIMEOUT RX] Đã quá 1s không nhận được gói DONE nào. Tự động thoát chu kỳ.\n");
                fflush(stdout);
                break;
            }
        }
        
        SET_CE(0); // Hạ chân CE về trạng thái an toàn
    }
    return NULL;
}

#define MODE_MANUAL 1
#define MODE_AUTO   2
int system_mode = MODE_MANUAL;
int robot_x = 0, robot_y = 0;

void* LAN_Network_Task(void* arg) {
    (void)arg; 

    // 1. Khởi tạo ZeroMQ
    void *context = zmq_ctx_new();
    void *subscriber = zmq_socket(context, ZMQ_SUB);
    zmq_connect(subscriber, "tcp://192.168.10.5:5555");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "robot1", 6);
    
    char input_buf[100];
    char zmq_buf[256];
    char topic[32];
    int selected_drone = 1; // Mặc định điều khiển Drone 1 ở Manual

    printf("\n=========================================================\n");
    printf("[LAN_TASK] HỆ THỐNG KHỞI CHẠY: CHẾ ĐỘ 1 - MANUAL (BÀN PHÍM) \n");

    while(1) {
        // NHÁNH 1: CHẾ ĐỘ MANUAL
        if (system_mode == MODE_MANUAL) {
            printf("\n[MANUAL - DRONE %d] Nhập lệnh: ", selected_drone);
            fflush(stdout);
            
            if (fgets(input_buf, sizeof(input_buf), stdin) == NULL) 
                continue;
            input_buf[strcspn(input_buf, "\n")] = '\0';

            if (strlen(input_buf) == 0) continue;

            // Lệnh chuyển trạng thái sang AUTO
            if (strcmp(input_buf, "auto") == 0) {
                system_mode = MODE_AUTO;
                printf("\n[HỆ THỐNG] >>> CHUYỂN SANG CHẾ ĐỘ 2: AUTO  <<<\n");
                printf("Mẹo bảo hiểm: Gõ 'manual' hoặc 'land' bất cứ lúc nào để ngắt khẩn cấp!\n");
                continue;
            }

            // Các lệnh chọn nhanh cấu hình địa chỉ Drone mục tiêu
            if (strcmp(input_buf, "drone1") == 0) {
                selected_drone = 1;
                memcpy(target_drone_addr, drone_addresses[0], 5);
                printf("[HỆ THỐNG] Đã chuyển mục tiêu sang điều khiển: DRONE 1 (0x%02X)\n", target_drone_addr[0]);
                continue;
            }
            if (strcmp(input_buf, "drone2") == 0) {
                selected_drone = 2;
                memcpy(target_drone_addr, drone_addresses[1], 5);
                printf("[HỆ THỐNG] Đã chuyển mục tiêu sang điều khiển: DRONE 2 (0x%02X)\n", target_drone_addr[0]);
                continue;
            }
            if (strcmp(input_buf, "drone3") == 0) {
                selected_drone = 3;
                memcpy(target_drone_addr, drone_addresses[2], 5);
                printf("[HỆ THỐNG] Đã chuyển mục tiêu sang điều khiển: DRONE 3 (0x%02X)\n", target_drone_addr[0]);
                continue;
            }
            if (strcmp(input_buf, "drone4") == 0) {
                selected_drone = 4;
                memcpy(target_drone_addr, drone_addresses[3], 5);
                printf("[HỆ THỐNG] Đã chuyển mục tiêu sang điều khiển: DRONE 4 (0x%02X)\n", target_drone_addr[0]);
                continue;
            }
            if (strcmp(input_buf, "drone5") == 0) {
                selected_drone = 5;
                memcpy(target_drone_addr, drone_addresses[4], 5);
                printf("[HỆ THỐNG] Đã chuyển mục tiêu sang điều khiển: DRONE 5 (0x%02X)\n", target_drone_addr[0]);
                continue;
            }

            // Đóng gói lệnh thủ công bình thường (LAND, TAKEOFF, v.v...) gửi đi
            strncpy(global_cmd, input_buf, sizeof(global_cmd) - 1);
            global_cmd[sizeof(global_cmd) - 1] = '\0';

            printf("\n---------------------------------------------------------");
            printf("\n[MANUAL] Đang phát lệnh '%s' tới DRONE %d...\n", global_cmd, selected_drone);
            
            sem_post(&sem_tx_request); // Đánh thức luồng RF bắn sóng nRF24L01

            // Treo luồng chờ phản hồi ACK từ Drone mục tiêu (Timeout 1 giây)
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1; 

            if (sem_timedwait(&sem_rx_complete, &ts) == 0) {
                printf("[MANUAL] -> THÀNH CÔNG: Nhận phản hồi 'DONE' từ Drone %d.\n", selected_drone);
            } else {
                printf("[MANUAL] -> TIMEOUT: Không thấy Drone %d báo cáo.\n", selected_drone);
            }
            printf("---------------------------------------------------------\n");
        }
        
        // ====================================================================
        // NHÁNH 2: CHẾ ĐỘ AUTO (ĐỌC MẠNG LAN) + PHANH KHẨN CẤP BÀN PHÍM
        // ====================================================================
        else if (system_mode == MODE_AUTO) {
            // Mặc định ở chế độ tự động, ta sẽ cấu hình địa chỉ bắn cho nhóm drone tự động
            // Khúc này bạn có thể map cứng drone nào chạy tự động tùy ý bạn, ví dụ:
            memcpy(target_drone_addr, drone_addresses[2], 5); 

            // ⚡ CƠ CHẾ KHÔNG CHẶN (ZMQ_DONTWAIT): Hứng gói tin nếu có, nếu không có lướt qua luôn
            int bytes_received = zmq_recv(subscriber, zmq_buf, sizeof(zmq_buf) - 1, ZMQ_DONTWAIT);
            
            if (bytes_received > 0) {
                zmq_buf[bytes_received] = '\0';
                
                int dummy_z; // Biến tạm để nuốt giá trị trục Z từ Pi 5 truyền sang để tránh lỗi hàm sscanf
                // Tách chuỗi thô: Chỉ lấy X và Y, loại bỏ hoàn toàn việc tính toán Z
                sscanf(zmq_buf, "%s %d %d %d", topic, &robot_x, &robot_y, &dummy_z);
                
                // Đóng gói chuỗi điều khiển tự động chỉ chứa 2 trục X, Y
                snprintf(global_cmd, sizeof(global_cmd), "GOTO %d %d", robot_x, robot_y);
                
                printf("[AUTO] Khớp ảnh Pi 5: X=%d, Y=%d -> Lệnh phát: '%s'\n", robot_x, robot_y, global_cmd);
                
                // Kích luồng nRF24L01 đẩy sóng đi thời gian thực
                sem_post(&sem_tx_request);
            }

            // ⚡ CƠ CHẾ NÊU TRÊN GIÚP GIẢI PHÓNG LUỒNG: Kiểm tra xem người dùng có gõ phím ngắt khẩn cấp không?
            // Sử dụng một hàm kiểm tra bộ đệm bàn phím không chặn nhẹ nhàng bằng tiếng C
            struct timeval tv = {0, 0}; // Không đợi (Timeout = 0)
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            
            select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
            
            // Nếu phát hiện có tín hiệu gõ phím từ bàn phím khi đang chạy AUTO!
            if (FD_ISSET(STDIN_FILENO, &fds)) {
                if (fgets(input_buf, sizeof(input_buf), stdin) != NULL) {
                    input_buf[strcspn(input_buf, "\n")] = '\0';
                    
                    // KỊCH BẢN 1: Gõ 'manual' -> Trả lại quyền điều khiển thủ công
                    if (strcmp(input_buf, "manual") == 0) {
                        system_mode = MODE_MANUAL;
                        printf("\n[HỆ THỐNG] <<< ĐÃ NGẮT AUTO -> TRẢ VỀ CHẾ ĐỘ MANUAL >>>\n");
                    }
                    // KỊCH BẢN 2: Drone gặp sự cố, gõ lệnh khẩn cấp như 'land' hoặc 'stop'
                    else if (strcmp(input_buf, "land") == 0 || strcmp(input_buf, "LAND") == 0) {
                        system_mode = MODE_MANUAL; // Ép về manual ngay lập tức
                        
                        strncpy(global_cmd, "LAND", sizeof(global_cmd) - 1);
                        global_cmd[sizeof(global_cmd) - 1] = '\0';
                        
                        printf("\n[HẠ CÁNH KHẨN CẤP] Ép ngắt Auto! Bắn lệnh 'LAND' cứu drone ngay lập tức!\n");
                        sem_post(&sem_tx_request); // Ưu tiên phát lệnh cứu hộ ngay
                    }
                }
            }
            
            // Tránh quá tải CPU cho vòng lặp không chặn của chế độ Auto
            usleep(10000); // Sleep 10ms (Chu kỳ quét phím & mạng là 100Hz, cực nhanh và nhạy)
        }
    }

    zmq_close(subscriber);
    zmq_ctx_destroy(context);
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
