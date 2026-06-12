Cách xài nrf51822 bằng st-link.

Xài 2 lệnh này ở 2 terminal khác nhau để xem nó còn sống không.

Terminal 1:  openocd -f interface/stlink.cfg -f target/nrf51.cfg
Terminal 2: telnet localhost 4444.
Kết quả:
<img width="742" height="627" alt="image" src="https://github.com/user-attachments/assets/9abd9475-79ae-4105-8e4e-4380cb6c0701" />

Kiểm tra giá trị thạch anh trước khi làm:
mdw 0x4000040C : Thạch anh - Giá trị là 00010001
<img width="742" height="201" alt="image" src="https://github.com/user-attachments/assets/7ec3bc01-6443-4158-99de-d0ab273c43aa" />

Nếu ra 00000000: Xử lý như sau đối với chip mua cũ hoặc rã mạch cũ mà bị lỗi phần cứng
Mở openocd -> telnet như trên
Bấm lệnh 
  + halt
  + nrf51 mass_erase
  + reset halt
  + mdw 0x40000408 -> phải ra 0
 Sau đó kiểm tra lại thạch anh -> Phải ra 00010000 -> Chứng tỏ thạch anh chạy
Nếu xài thạch anh ngoài 16MHz

mww 0x40000508 0xFF     # XTALFREQ = 16MHz (hoặc 0x00 cho 32MHz)
mww 0x40000000 1        # TASKS_HFCLKSTART
sleep 100
mdw 0x40000408          # Phải ra 1 (EVENTS_HFCLKSTARTED)
mdw 0x4000040C          # Kết quả sẽ ra 00010001


Nếu dùng radio để nghe packet
mdw 0x40001100 : Radio đã vào trạng thái hay chưa -> = 1 là oke
mdw 0x40001550 : State của radio -> =3 là đang nghe sóng
có cảm giác sai mdw 0x4000110C : Bắt sóng của radio -> = 1 là đã bắt được sóng
mdw 0x40001110 : Gói package -> =1 là đã nhận

mdw 0x40001548 : RSSI khác 0
mdw 0x4000111C : CRC check
mdw 0x40001400 : CRC status

mdw 0x4000151C : BASE 0
mdw 0x40001524 : PREFIX 0
mdw 0x40001514 : PCNF 0
mdw 0x40001518 : PCNF1

mdw 0x40001504 : Kiểm tra vị trí được nhét package -> Kết quả (Z)

mdb 0xKết quả (Z) 16
