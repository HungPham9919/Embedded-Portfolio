# ADS1115
Some brief introductions about ADS1115 (ADC)  
The ADS1115 is a product developed by Texas Instruments (TI), a major American semiconductor company known for producing analog and embedded processing chips.  
The ADS1115 was officially introduced by TI in August 2010.  
The ADS1115 is a high-precision, low-power, 16-bit analog-to-digital converter (ADC)  
For particular:  
  - Resolution: 16-bit (Maximun value is 65535)  
  - Input channel: 4 channels  
  - Conmunication: I2C  
  - Voltage supply (VDD): 2.0V-5.5V  
  - Input range: +-(0.2567 - 6,144V)  
  - Sampling Rate: Up to 860 SPS  
  - PGA options: +- (6.144 - 4.096 - ...)  
  
  In this situation, i use Raspberrypi model 3B+, ADS1115 and LM35 (Temperature sensor)  
  - ADS1115 VCC: 1st Pi's header  
  - SDA: 3rd Pi's header  
  - SCL: 5st Pi's header  
  - Ground: 14th Pi's header  
  - AN0: from LM35  
  
  Some information about ADS1115 (How to read and build its register)  
  DataSheet: https://www.alldatasheet.com/datasheet-pdf/view/292735/TI/ADS1115.html
  ![image](https://github.com/user-attachments/assets/8f638d61-7dcc-4372-b503-059c0dca0f17)  
  ![image](https://github.com/user-attachments/assets/d09045aa-b596-4b78-b122-82e2f3c66c83)  
  ![image](https://github.com/user-attachments/assets/23fcf1d6-e930-4e5f-84ea-875dc8411cdc)  
  Some notes:  
    + This is my pi's version: 6.6.74+rpt-rpi-v7 (You can use this command to check your version: "uname -r".  
    + You can update your version by using this command "sudo apt update"  
    + If you wanna upgrade the package "sudo apt full-upgrade"  
  (*) You need to check your address of your ads1115 by using "i2cdetect -y 1"  

  Before starting: You need to do some steps like that:  
   + "cd ~"  
   + "cd /boot/firmware" and see your pi hardware  
    ![image](https://github.com/user-attachments/assets/78519962-3b57-4c01-9cff-70bfc117ff7a)  
    + In my pi models is file bcm2710-rpi-3-b-plus.dtb (device tree binary)  
         You need to convert it into bcm2710-rpi-3-b-plus.dts (device tree script) by using "sudo dtc -I dtb -O dts -o bcm2710-rpi-3-b-plus.dts bcm2710-rpi-3-b-plus.dtb"  
     In file script you can open it in some softwares that you have (Geany)  
     After that: You will find the I2C address "i2c@7e804000"  
     You will need to provide some informations like that:  
     ![image](https://github.com/user-attachments/assets/993f4335-eab3-4c91-9e21-6ba2c7603e1f)  
     Save it and convert in into file dts by using the command "sudo dtc -I dts -O dtb -o bcm2710-rpi-3-b-plus.dtb bcm2710-rpi-3-b-plus.dts"  
     At the end, you need to reboot it by using "sudo reboot"   

 Some libraries were used in driver.c  
  ![image](https://github.com/user-attachments/assets/df8fc6eb-b193-44e3-b06b-e7577e77663f)  
-> If you wanna see what i write for this part -> click on ads1115_driver.c to see all what i write.  

The second part: (file_ioctl.c)  
The libraries which were used in this part  
  ![image](https://github.com/user-attachments/assets/7e50ee1b-cdea-4278-aaab-ef72a913b574)  
  -> If you wanna see what i write for this part -> click on ads1115_ioctl.c to see all what i write.  

  Now, i will show you how to use and demo what it can do.  
  Note: All files stay in a folder  
  Firstly, in "Makefile" i need changing the obj += = ads_1115driver.o  
  Next: in termimal you write "make" (If you have the version and package like me.)  
  The result is:  
  ![image](https://github.com/user-attachments/assets/29deb96f-a37d-4d01-aad2-d89a3bb1f5bc)  
  After that, you will install the driver by using "sudo insmod ads1115_driver.ko"

In the next stage: In file ads1115_ioctl.c you need to rewrite the Makefile: "obj-m += ads1115_ioctl.o" and write make into terminal  
The result is:  
![image](https://github.com/user-attachments/assets/01766c16-de9d-4016-8a8c-73df8cc60f74)  
After that, you will install the driver by using "sudo insmod ads1115_ioctl.ko"  
/* You need to check that you had register right or wrong  
Using this command: "cat /proc/devices"  
The outcome:  
![image](https://github.com/user-attachments/assets/7b69e3c8-049c-4f3a-b455-e4611787a63d)  
Using this command: "ls /dev"  
The outcome:  
![image](https://github.com/user-attachments/assets/02a50bec-2ef4-4765-bfd0-6d526bb26df8)  
Using this command: "ls /sys/class"  
![image](https://github.com/user-attachments/assets/16c19a3e-487a-4f11-b7a0-dc13f5361275)  

If it appeared like that it means you right and if it's not (do it again and find you error)*/  

Now, i will demo the code that i write in user:  
You need compiling it "gcc -o 1 ads1115_user.c" -> "sudo ./1"  
![image](https://github.com/user-attachments/assets/eb2e3b2a-2dfe-47ea-a03e-10c4e4df8611)  
You just see the result of analog 1  
/*To remove the files which made in folder use "make clean in terminal"*/

