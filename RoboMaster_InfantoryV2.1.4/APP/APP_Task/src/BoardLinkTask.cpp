#include "BoardLinkTask.hpp"
#include "FreeRTOS.h"
#include "app_Data_Task.hpp"
#include "main.h"
#include "queue.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <string.h>

typedef struct {
    uint16_t size;
    uint8_t data[BOARD_LINK_MAX_FRAME_SIZE];
} BoardLinkRxPacket_t;

static QueueHandle_t board_link_rx_queue;
static uint16_t board_link_sequence;
uint8_t BoardLink_DmaRxBuffer[BOARD_LINK_RX_BUFFER_SIZE]
    __attribute__((aligned(32), section(".sram4")));

volatile BoardLink_GimbalState_t BoardLink_LastGimbalState;
volatile uint32_t BoardLink_RxOkCount;
volatile uint32_t BoardLink_RxErrorCount;

void BoardLink_RestartRx(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, BoardLink_DmaRxBuffer,
                                 sizeof(BoardLink_DmaRxBuffer));
}

void BoardLink_OnRx(const uint8_t *data, uint16_t size)
{
    if (board_link_rx_queue == NULL || data == NULL || size == 0U)
        return;
    BoardLinkRxPacket_t packet = {};
    packet.size = size > BOARD_LINK_MAX_FRAME_SIZE ? BOARD_LINK_MAX_FRAME_SIZE : size;
    memcpy(packet.data, data, packet.size);
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(board_link_rx_queue, &packet, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void BoardLink_ProcessRx(const BoardLinkRxPacket_t *packet)
{
    uint16_t payload_len = 0U;
    if (!BoardLink_Validate(packet->data, packet->size,
                            BOARD_LINK_SENDER_GIMBAL,
                            BOARD_LINK_TYPE_GIMBAL_STATE, &payload_len) ||
        payload_len != sizeof(BoardLink_GimbalState_t)) {
        ++BoardLink_RxErrorCount;
        return;
    }
    memcpy((void *)&BoardLink_LastGimbalState,
           packet->data + BOARD_LINK_HEADER_SIZE, sizeof(BoardLink_GimbalState_t));
    ++BoardLink_RxOkCount;
}

void BoardLinkTask(void *pvParameters)
{
    (void)pvParameters;
    board_link_rx_queue = xQueueCreate(4U, sizeof(BoardLinkRxPacket_t));
    if (board_link_rx_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }
    BoardLink_RestartRx();
    BoardLinkRxPacket_t packet;
    uint8_t tx_frame[BOARD_LINK_MAX_FRAME_SIZE];
    for (;;) {
        while (xQueueReceive(board_link_rx_queue, &packet, 0U) == pdPASS)
            BoardLink_ProcessRx(&packet);

        BoardLink_ChassisState_t state = {};
        state.timestamp_us = __HAL_TIM_GET_COUNTER(&htim5);
        state.roll = Classic_Data.IMU_Data.roll;
        state.pitch = Classic_Data.IMU_Data.pitch;
        state.yaw = Classic_Data.IMU_Data.yaw;
        state.delta_pitch = Classic_Data.IMU_Data.delta_pitch;
        state.delta_yaw = Classic_Data.IMU_Data.delta_yaw;
        state.leg_length_l = Classic_Data.Leg_Data.L0_L;
        state.leg_length_r = Classic_Data.Leg_Data.L0_R;
        state.wheel_velocity_l = Classic_Data.Motor_Data.Left_Wheel_Motor_VEL;
        state.wheel_velocity_r = Classic_Data.Motor_Data.Right_Wheel_Motor_VEL;
        state.target_x_velocity = Classic_Data.Target_Data.Target_X_vel;
        state.target_yaw_velocity = Classic_Data.Target_Data.Target_yaw_vel;
        state.control_flags = (uint8_t)(Classic_Data.Contronller_Data.F0_L != 0.0f);
        uint16_t tx_size = BoardLink_Build(tx_frame, BOARD_LINK_SENDER_CHASSIS,
                                           BOARD_LINK_TYPE_CHASSIS_STATE,
                                           board_link_sequence++, &state, sizeof(state));
        if (tx_size != 0U)
            HAL_UART_Transmit(&huart3, tx_frame, tx_size, 5U);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
