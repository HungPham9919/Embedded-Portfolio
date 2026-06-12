Cách xài nrf51822 bằng st-link.

Xài 2 lệnh này ở 2 terminal khác nhau để xem nó còn sống không<br>

Terminal 1:  openocd -f interface/stlink.cfg -f target/nrf51.cfg <br>
Terminal 2: telnet localhost 4444 <br>
Kết quả: <br>
<img width="742" height="627" alt="image" src="https://github.com/user-attachments/assets/9abd9475-79ae-4105-8e4e-4380cb6c0701" />
<br>
Kiểm tra giá trị thạch anh trước khi làm:<br>
mdw 0x4000040C : Thạch anh - Giá trị là 00010001<br>
<img width="742" height="201" alt="image" src="https://github.com/user-attachments/assets/7ec3bc01-6443-4158-99de-d0ab273c43aa" />
<br>
Nếu ra 00000000: Xử lý như sau đối với chip mua cũ hoặc rã mạch cũ mà bị lỗi phần cứng<br>
Mở openocd -> telnet như trên<br>
Bấm lệnh <br>
  + halt
  + nrf51 mass_erase
  + reset halt
  + mdw 0x40000408 -> phải ra 0
 Sau đó kiểm tra lại thạch anh -> Phải ra 00010000 -> Chứng tỏ thạch anh chạy <br>
Nếu xài thạch anh ngoài 16MHz<br>

mww 0x40000508 0xFF     # XTALFREQ = 16MHz (hoặc 0x00 cho 32MHz)<br>
mww 0x40000000 1        # TASKS_HFCLKSTART<br>
sleep 100
mdw 0x40000408          # Phải ra 1 (EVENTS_HFCLKSTARTED)<br>
mdw 0x4000040C          # Kết quả sẽ ra 00010001<br>


Nếu dùng radio để nghe packet<br>
mdw 0x40001100 : Radio đã vào trạng thái hay chưa -> = 1 là oke<br>
mdw 0x40001550 : State của radio -> =3 là đang nghe sóng<br>
có cảm giác sai mdw 0x4000110C : Bắt sóng của radio -> = 1 là đã bắt được sóng<br>
mdw 0x40001110 : Gói package -> =1 là đã nhận<br>

mdw 0x40001548 : RSSI khác 0<br>
mdw 0x4000111C : CRC check<br>
mdw 0x40001400 : CRC status<br>

mdw 0x4000151C : BASE 0 <br>
mdw 0x40001524 : PREFIX 0 <br>
mdw 0x40001514 : PCNF 0 <br>
mdw 0x40001518 : PCNF1 <br>

mdw 0x40001504 : Kiểm tra vị trí được nhét package -> Kết quả (Z) <br>

mdb 0xKết quả (Z) 16 <br>
