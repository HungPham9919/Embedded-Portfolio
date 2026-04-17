#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h> // Include errno header
#include "unistd.h"

#define ADS1115_PATH "/dev/ads1115"
#define ADS1115_IOCTL_MAGIC 'A'
#define ADS1115_IOCTL_READ_AN0 _IOR(ADS1115_IOCTL_MAGIC,1,int)
#define ADS1115_IOCTL_READ_AN1 _IOR(ADS1115_IOCTL_MAGIC,2,int)
#define ADS1115_IOCTL_READ_AN2 _IOR(ADS1115_IOCTL_MAGIC,3,int)
#define ADS1115_IOCTL_READ_AN3 _IOR(ADS1115_IOCTL_MAGIC,4,int)

int main(void){
    int fd, data;

    fd = open(ADS1115_PATH, O_RDONLY);
    if(fd < 0){
        perror("Failed to open device \n");
        return errno;
    }

    // Read the 1st channel
    if(ioctl(fd,ADS1115_IOCTL_READ_AN0,&data) < 0){
        perror("Failed to read Analog_1 \n");
        close(fd);
        return errno;
    }
        printf("The value of analog_1 is %d \n", data);
    // second channel
    if(ioctl(fd,ADS1115_IOCTL_READ_AN1,&data) < 0){
        perror("Failed to read Analog_2 \n");
        close(fd);
        return errno;
    } 
    printf("The value of analog_2 is %d \n", data);
    // third channel
    if(ioctl(fd,ADS1115_IOCTL_READ_AN2,&data) < 0){
        perror("Failed to read Analog_3 \n");
        close(fd);
        return errno;
    }

    printf("The value of analog_3 is %d \n", data);
    // last one
    if(ioctl(fd,ADS1115_IOCTL_READ_AN3,&data) < 0){
        perror("Failed to read Analog_1 \n");
        close(fd);
        return errno;
    }
    printf("The value of analog_4 is %d \n", data);
    // close the dvice
    close(fd);
    return 0;
}
