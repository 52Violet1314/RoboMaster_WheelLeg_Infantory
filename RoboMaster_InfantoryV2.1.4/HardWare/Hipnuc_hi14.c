
/*
 * Copyright (c) 2006-2024, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "Hipnuv_hi14.h"

hi14_t Hipnuc_HI14;

/* ─── DMA 缓冲区：必须放在 RAM_D1（DMA 可达），不能放 DTCM ─── */
uint8_t hi14_dma_rx_buf[UART_RX_BUF_SIZE]
    __attribute__((aligned(32), section(".sram4")));

/* ─── 处理缓冲区：仅在 CPU 域使用，可放在 DTCM ─── */
uint8_t hi14_uart_rx_buf[UART_RX_BUF_SIZE];

/* ISR 与主循环共享 */
uint16_t hi14_rx_size = 0;

/* The driver file for decoding HiPNUC protocol, DO NOT MODIFTY*/

/* HiPNUC protocol constants */
#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)              /* CHAOHE protocol header size */


/* new HiPNUC standard packet */
#define HIPNUC_ID_HI91        (0x91)

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif


static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len);

/* common type conversion */
#define I2(p) (*((int16_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

static uint32_t U4(uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}

static double D8(uint8_t *p)
{
    double d;
    memcpy(&d, p, 8);
    return d;
}

static uint64_t U8(uint8_t *p)
{
    uint64_t u;
    memcpy(&u, p, 8);
    return u;
}

/* parse the payload of a frame and feed into data section */
static int parse_data(hi14_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    
    /* ignore all previous data */
    raw->hi14data.tag = 0;

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_ID_HI91:
            memcpy(&raw->hi14data, p + ofs, sizeof(hi14_data_t));
            ofs += sizeof(hi14_data_t);
            break;
        default:
            ofs++;
            break;
        }
    }
    return 1;
}

static int decode_hipnuc(hi14_t *raw)
{
    uint16_t crc = 0;

    /* checksum */
    hipnuc_crc16(&crc, raw->buf, (CH_HDR_SIZE-2));
    hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
    if (crc != U2(raw->buf + (CH_HDR_SIZE-2)))
    {
        printf("ch checksum error: frame:0x%04X calculated:0x%04X, len:%d\n", (unsigned)U2(raw->buf + CH_HDR_SIZE - 2), (unsigned)crc, raw->len);
        return -1;
    }

    return parse_data(raw);
}

/* sync code */
static int sync_hipnuc(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

/**
 * @brief     HiPNUC decoder input, read one byte at a time.
 *
 * @param    raw is the decoder struct.
 * @param    data is the one byte read from stream.
 * @return   >0: decoder received a frame successfully, else: receiver did not receive a frame successfully.
 */
int hipnuc_input(hi14_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_hipnuc(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE))
        {
            printf("ch length error: len=%d\n", raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        return 0;
    }

    raw->nbyte = 0;

    return decode_hipnuc(raw);
}


/**
 * @brief    Convert packet to string, only dump parts of data
 *
 * @param    raw is struct of decoder
 * @param    buf is the log string buffer, make sure buf is larger than 256
 * @param    buf_size is the size of the log buffer
 * @return   Number of characters written to the buffer
 */
int hipnuc_dump_packet(hi14_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret;

    /* dump 0x91 packet */
    if(raw->hi14data.tag == HIPNUC_ID_HI91)
    {
        /* Format:
         * system_time: ms
         * acc: m/s²
         * gyr: deg/s
         * mag: uT
         * pitch/roll/yaw: deg
         * quat: w,x,y,z
         * air_pressure: Pa
         */
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"hi14data\",\n"
            "  \"main_status\": [0x%X],\n"
            "  \"system_time\": %lu,\n"
            "  \"acc\": [%.3f, %.3f, %.3f],\n"
            "  \"gyr\": [%.3f, %.3f, %.3f],\n"
            "  \"mag\": [%.3f, %.3f, %.3f],\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"yaw\": %.2f,\n"
            "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
            "  \"air_pressure\": %.1f\n"
            "}\n",
            (unsigned)raw->hi14data.main_status,
            (unsigned long)raw->hi14data.system_time,
            raw->hi14data.acc[0]*GRAVITY, raw->hi14data.acc[1]*GRAVITY, raw->hi14data.acc[2]*GRAVITY,
            raw->hi14data.gyr[0], raw->hi14data.gyr[1], raw->hi14data.gyr[2],
            raw->hi14data.mag[0], raw->hi14data.mag[1], raw->hi14data.mag[2],
            raw->hi14data.pitch, raw->hi14data.roll, raw->hi14data.yaw,
            raw->hi14data.quat[0], raw->hi14data.quat[1], raw->hi14data.quat[2], raw->hi14data.quat[3],
            raw->hi14data.air_pressure);
    }
    if (ret > 0) written += ret;
    return written;
}

/**
 * @brief    Calculate HiPNUC CRC16
 *
 * @param    inital is initial value
 * @param    buf    is input buffer pointer
 * @param    len    is length of the buffer
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *inital = crc;
}
