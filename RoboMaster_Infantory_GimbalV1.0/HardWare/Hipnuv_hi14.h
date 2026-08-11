#ifndef __HIPNUUV_HI14_H__
#define __HIPNUUV_HI14_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* UART7 DMA 接收缓冲区大小 */
#define UART_RX_BUF_SIZE        1024

/* 日志输出字符串缓冲区大小 */
#define LOG_STRING_SIZE         1024


#define HIPNUC_MAX_RAW_SIZE     (512)
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        main_status;    /* reserved */
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           quat[4];        /* Quaternion (w, x, y, z) */
} hi14_data_t;

typedef struct
{
    int nbyte;                          /* Number of bytes in message buffer */ 
    int len;                            /* Message length (bytes) */
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];   /* Message raw buffer */
    hi14_data_t hi14data;
} hi14_t;


extern hi14_t Hipnuc_HI14;
extern uint8_t hi14_uart_rx_buf[UART_RX_BUF_SIZE];
extern uint8_t hi14_dma_rx_buf[UART_RX_BUF_SIZE];
extern uint16_t hi14_rx_size;

/**
 * @brief Process one byte of input data for HiPNUC decoder
 * @param raw Pointer to hi14_t structure
 * @param data Input byte to process
 * @return 1 if a complete packet was successfully decoded, 0 if more data is needed, -1 on error
 */
int hipnuc_input(hi14_t *raw, uint8_t data);

/**
 * @brief Dump decoded HiPNUC packet data to a string buffer
 * @param raw Pointer to hi14_t structure containing decoded data
 * @param buf Output buffer to store the formatted string
 * @param buf_size Size of the output buffer
 * @return Number of characters written to the buffer
 */
int hipnuc_dump_packet(hi14_t *raw, char *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif