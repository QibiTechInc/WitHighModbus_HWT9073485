#include "wit_c_sdk.h"
#include "REG.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#define ACC_UPDATE      0x01
#define GYRO_UPDATE     0x02
#define ANGLE_UPDATE    0x04
#define MAG_UPDATE      0x08
#define TEMP_UPDATE     0x10
#define READ_UPDATE     0x80

static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

const int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

// Function prototypes
static int serial_open(const char *port, int baud);
static int serial_read_data(int fd, char *buf, int size);
static int serial_write_data(int fd, const uint8_t *p_data, uint32_t uiSize);
static void serial_close(int fd);
static void AutoScanSensor(char *dev);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void Delayms(uint16_t ucMs);

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s /dev/ttyUSB0\n", argv[0]);
        return 0;
    }

    // Attempt to open the serial port
    if ((fd = serial_open(argv[1], s_iCurBaud)) < 0) {
        printf("Error: Could not open serial port %s\n", argv[1]);
        return -1;
    }
    printf("Successfully opened %s\n", argv[1]);

    float fAcc[3], fGyro[3], fAngle[3], fTemp;
    int i, iBuff;
    char cBuff[1];

    WitInit(WIT_PROTOCOL_905x_MODBUS, 0xff);
    WitRegisterCallBack(CopeSensorData);
    WitSerialWriteRegister(SensorUartSend);

    printf("\r\n********************** wit-motion HWT905x-MODBUS Example ************************\r\n");
    AutoScanSensor(argv[1]);

    while (1) {
        WitReadReg(AX, 16);
        Delayms(500);

        while (serial_read_data(fd, cBuff, 1) > 0) {
            WitSerialDataIn(cBuff[0]);
        }

        if (s_cDataUpdate) {
            for (i = 0; i < 3; i++) {
                fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
                fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
                iBuff = (((uint32_t)sReg[HRoll + 2 * i]) << 16) | ((uint16_t)sReg[LRoll + 2 * i]);
                fAngle[i] = (float)iBuff / 1000.0f;
            }
            fTemp = (float)sReg[TEMP905x] / 100.0f;

            if (s_cDataUpdate & ACC_UPDATE) {
                printf("Acceleration: %.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
                s_cDataUpdate &= ~ACC_UPDATE;
            }
            if (s_cDataUpdate & GYRO_UPDATE) {
                printf("Gyroscope: %.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
                s_cDataUpdate &= ~GYRO_UPDATE;
            }
            if (s_cDataUpdate & ANGLE_UPDATE) {
                printf("Angle: %.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
                s_cDataUpdate &= ~ANGLE_UPDATE;
            }
            if (s_cDataUpdate & MAG_UPDATE) {
                printf("Magnetic Field: %d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
                s_cDataUpdate &= ~MAG_UPDATE;
            }
            if (s_cDataUpdate & TEMP_UPDATE) {
                printf("Temperature: %.1f\r\n", fTemp);
                s_cDataUpdate &= ~TEMP_UPDATE;
            }
             s_cDataUpdate = 0;
        }
    }

    serial_close(fd);
    return 0;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
    for (uint32_t i = 0; i < uiRegNum; i++) {
        switch (uiReg + i) {
            case AZ: s_cDataUpdate |= ACC_UPDATE; break;
            case GZ: s_cDataUpdate |= GYRO_UPDATE; break;
            case HZ: s_cDataUpdate |= MAG_UPDATE; break;
            case Yaw: s_cDataUpdate |= ANGLE_UPDATE; break;
            case TEMP905x: s_cDataUpdate |= TEMP_UPDATE; break;
            default: s_cDataUpdate |= READ_UPDATE; break;
        }
    }
}

static void Delayms(uint16_t ucMs) {
    usleep(ucMs * 1000);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    serial_write_data(fd, p_data, uiSize);
    // The delay might still be necessary depending on the device's response time.
    usleep(((1000000 / (s_iCurBaud / 10)) * uiSize) + 300);
}

static void AutoScanSensor(char *dev) {
    char cBuff[1];
    for (size_t i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
        serial_close(fd);
        s_iCurBaud = c_uiBaud[i];
        fd = serial_open(dev, s_iCurBaud);
        if (fd < 0) continue;

        s_cDataUpdate = 0;
        WitReadReg(AX, 3);
        Delayms(200);

        while (serial_read_data(fd, cBuff, 1) > 0) {
            WitSerialDataIn(cBuff[0]);
        }

        if (s_cDataUpdate != 0) {
            printf("Sensor found at %d baud.\r\n\r\n", s_iCurBaud);
            return;
        }
    }
    printf("Could not find the sensor. Please check the connection.\r\n");
    exit(-1);
}

// Pure PC Linux implementation for serial communication
static int serial_open(const char *port, int baud) {
    int local_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (local_fd == -1) {
        perror("serial_open: Unable to open port");
        return -1;
    }

    struct termios options;
    tcgetattr(local_fd, &options);

    speed_t baud_rate;
    switch (baud) {
        case 2400: baud_rate = B2400; break;
        case 4800: baud_rate = B4800; break;
        case 9600: baud_rate = B9600; break;
        case 19200: baud_rate = B19200; break;
        case 38400: baud_rate = B38400; break;
        case 57600: baud_rate = B57600; break;
        case 115200: baud_rate = B115200; break;
        case 230400: baud_rate = B230400; break;
        case 460800: baud_rate = B460800; break;
        case 921600: baud_rate = B921600; break;
        default: return -1;
    }
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1; // 0.1 seconds read timeout

    tcsetattr(local_fd, TCSANOW, &options);
    return local_fd;
}

static int serial_read_data(int fd, char *buf, int size) {
    return read(fd, buf, size);
}

static int serial_write_data(int fd, const uint8_t *p_data, uint32_t uiSize) {
    return write(fd, p_data, uiSize);
}

static void serial_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}