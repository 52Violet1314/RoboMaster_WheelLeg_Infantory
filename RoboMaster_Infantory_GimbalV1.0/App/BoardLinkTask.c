#include "BoardLinkTask.h"
#include "FreeRTOS.h"
#include "Hipnuv_hi14.h"
#include "gimbal.h"
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
    __attribute__((aligned(32), section(".RAM_D1_NC")));

volatile BoardLink_ChassisState_t BoardLink_LastChassisState;
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
    BoardLinkRxPacket_t packet = {0};
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
                            BOARD_LINK_SENDER_CHASSIS,
                            BOARD_LINK_TYPE_CHASSIS_STATE, &payload_len) ||
        payload_len != sizeof(BoardLink_ChassisState_t)) {
        ++BoardLink_RxErrorCount;
        return;
    }
    memcpy((void *)&BoardLink_LastChassisState,
           packet->data + BOARD_LINK_HEADER_SIZE, sizeof(BoardLink_ChassisState_t));
    ++BoardLink_RxOkCount;
}

void BoardLinkTask(void *argument)
{
    (void)argument;
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

        BoardLink_GimbalState_t state = {0};
        state.timestamp_us = __HAL_TIM_GET_COUNTER(&htim5);
        state.roll = Hipnuc_HI14.hi14data.roll;
        state.pitch = Hipnuc_HI14.hi14data.pitch;
        state.yaw = Hipnuc_HI14.hi14data.yaw;
        state.target_pitch = gimbal_recv_state.pitch;
        state.control_flags = gimbal_recv_state.mode;
        state.status = 1U;
        uint16_t tx_size = BoardLink_Build(tx_frame, BOARD_LINK_SENDER_GIMBAL,
                                           BOARD_LINK_TYPE_GIMBAL_STATE,
                                           board_link_sequence++, &state, sizeof(state));
        if (tx_size != 0U)
            HAL_UART_Transmit(&huart3, tx_frame, tx_size, 5U);

        if (hi14_rx_size > 0U) {
            for (uint16_t i = 0U; i < hi14_rx_size; ++i) {
                if (hipnuc_input(&Hipnuc_HI14, hi14_uart_rx_buf[i]) > 0)
                    usb_send_gimbal_state(Hipnuc_HI14.hi14data.roll,
                                           Hipnuc_HI14.hi14data.pitch,
                                           Hipnuc_HI14.hi14data.yaw);
            }
            hi14_rx_size = 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
