#define _GNU_SOURCE        
#define _POSIX_C_SOURCE 200112L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <signal.h>
#include "pthread.h"
#include "semaphore.h"
#include "unistd.h"
#include "time.h"

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07

#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F

#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define PAYLOAD_WIDTH 0x60

#define W_TX_PAYLOAD  0xA0
#define TX_ADDR       0x10


