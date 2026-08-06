// Đọc giá trị tại một ô nhớ tạm (ví dụ địa chỉ 0x00 hoặc 0xFF).

//     Ghi một giá trị test ngẫu nhiên (ví dụ 0xA5) vào ô nhớ đó.

//     Đọc lại ô nhớ đó xem có đúng 0xA5 hay không.

//     Ghi lại giá trị ban đầu để khôi phục dữ liệu gốc.

// C

// bool check_at24c02(uint8_t dev_addr) {
//     uint8_t test_addr = 0xFE; // Chọn 1 ô nhớ gần cuối
//     uint8_t original_val, read_val;
//     uint8_t test_pattern = 0xA5;

//     // 1. Đọc lưu lại giá trị gốc
//     original_val = eeprom_read_byte(dev_addr, test_addr);

//     // 2. Ghi byte test
//     eeprom_write_byte(dev_addr, test_addr, test_pattern);
//     delay_ms(5); // Chờ thời gian write cycle (t_WR) của EEPROM

//     // 3. Đọc lại để xác nhận
//     read_val = eeprom_read_byte(dev_addr, test_addr);

//     // 4. Khôi phục lại byte gốc
//     eeprom_write_byte(dev_addr, test_addr, original_val);
//     delay_ms(5);

//     return (read_val == test_pattern);
// }