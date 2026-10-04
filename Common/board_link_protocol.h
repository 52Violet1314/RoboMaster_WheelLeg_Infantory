#ifndef BOARD_LINK_PROTOCOL_H
#define BOARD_LINK_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define BOARD_LINK_BAUDRATE       2000000U
#define BOARD_LINK_SOF0            0xA5U
#define BOARD_LINK_SOF1            0x5AU
#define BOARD_LINK_VERSION         1U
#define BOARD_LINK_MAX_PAYLOAD     64U
#define BOARD_LINK_RX_BUFFER_SIZE  128U
#define BOARD_LINK_HEADER_SIZE      9U
#define BOARD_LINK_MAX_FRAME_SIZE  (BOARD_LINK_HEADER_SIZE + BOARD_LINK_MAX_PAYLOAD + 2U)

enum { BOARD_LINK_SENDER_CHASSIS = 1U, BOARD_LINK_SENDER_GIMBAL = 2U };
enum { BOARD_LINK_TYPE_CHASSIS_STATE = 1U, BOARD_LINK_TYPE_GIMBAL_STATE = 2U };

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    float roll, pitch, yaw;                 /* rad */
    float delta_pitch, delta_yaw;            /* rad/s */
    float leg_length_l, leg_length_r;        /* m */
    float wheel_velocity_l, wheel_velocity_r;/* m/s */
    float target_x_velocity, target_yaw_velocity; /* m/s, rad/s */
    uint8_t control_flags;
    uint8_t reserved[3];
} BoardLink_ChassisState_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    float roll, pitch, yaw;                 /* degrees, as reported by HiPNUC */
    float target_pitch;                     /* degrees, USB command convention */
    uint8_t control_flags;
    uint8_t status;
    uint8_t reserved[2];
} BoardLink_GimbalState_t;

static inline uint16_t BoardLink_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    while (length-- != 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0x8408U)
                              : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static inline uint16_t BoardLink_Build(uint8_t *frame, uint8_t sender,
                                       uint8_t type, uint16_t sequence,
                                       const void *payload, uint16_t payload_len)
{
    if (frame == NULL || payload == NULL || payload_len > BOARD_LINK_MAX_PAYLOAD)
        return 0U;
    frame[0] = BOARD_LINK_SOF0; frame[1] = BOARD_LINK_SOF1;
    frame[2] = BOARD_LINK_VERSION; frame[3] = sender; frame[4] = type;
    frame[5] = (uint8_t)payload_len; frame[6] = (uint8_t)(payload_len >> 8U);
    frame[7] = (uint8_t)sequence; frame[8] = (uint8_t)(sequence >> 8U);
    memcpy(frame + BOARD_LINK_HEADER_SIZE, payload, payload_len);
    uint16_t crc = BoardLink_Crc16(frame, (uint16_t)(BOARD_LINK_HEADER_SIZE + payload_len));
    frame[BOARD_LINK_HEADER_SIZE + payload_len] = (uint8_t)crc;
    frame[BOARD_LINK_HEADER_SIZE + payload_len + 1U] = (uint8_t)(crc >> 8U);
    return (uint16_t)(BOARD_LINK_HEADER_SIZE + payload_len + 2U);
}

static inline int BoardLink_Validate(const uint8_t *frame, uint16_t frame_len,
                                     uint8_t expected_sender, uint8_t expected_type,
                                     uint16_t *payload_len)
{
    if (frame == NULL || frame_len < BOARD_LINK_HEADER_SIZE + 2U ||
        frame[0] != BOARD_LINK_SOF0 || frame[1] != BOARD_LINK_SOF1 ||
        frame[2] != BOARD_LINK_VERSION || frame[3] != expected_sender ||
        frame[4] != expected_type)
        return 0;
    uint16_t length = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8U);
    uint16_t expected_len = (uint16_t)(BOARD_LINK_HEADER_SIZE + length + 2U);
    if (length > BOARD_LINK_MAX_PAYLOAD || frame_len != expected_len)
        return 0;
    uint16_t received_crc = (uint16_t)frame[expected_len - 2U] |
                            ((uint16_t)frame[expected_len - 1U] << 8U);
    if (BoardLink_Crc16(frame, (uint16_t)(expected_len - 2U)) != received_crc)
        return 0;
    if (payload_len != NULL) *payload_len = length;
    return 1;
}

#endif
