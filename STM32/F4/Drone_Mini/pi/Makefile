TARGET = gateway_app
CC = gcc
CFLAGS = -Wall -Wextra -Ofast
LIBS = -lgpiod -lpthread -lrt

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

# Quy tắc mặc định khi gõ 'make'
all: $(TARGET)
	@echo "============================================="
	@echo "Đang cấp quyền thực thi cho $(TARGET)..."
	chmod +x $(TARGET)
	@echo "Đã biên dịch thành công ứng dụng với gpiod V2 & Pthread!"
	@echo "Chạy chương trình bằng lệnh: sudo ./$(TARGET)"
	@echo "============================================="

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
	@echo "Đã xóa toàn bộ file tạm và file thực thi."

.PHONY: all clean
