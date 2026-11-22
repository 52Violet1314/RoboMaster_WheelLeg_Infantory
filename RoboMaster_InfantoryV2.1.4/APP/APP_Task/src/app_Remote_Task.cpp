#include "app_Remote_Task.hpp"
#include "app_Data_Task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_UART.h"
#include <cstdint>

static int16_t Remote_ApplyStickDeadzone(int16_t channel) {
    return (channel > 5 || channel < -5) ? channel : 0;
}

static void Remote_ParseControlSnapshot(Remote_ControlSnapshot_t *snapshot,
                                        const SBUS_Data_t *sbus) {
    constexpr float kRemoteChannelFullScale = 783.0f;
    constexpr float kCommandFullScale = 4.0f;
    /* 3519 额定轮端转速对应约 2.40 m/s；遥控满杆不能要求超过持续能力。 */
    constexpr float kLongitudinalVelocityFullScaleMps = 2.4f;

    snapshot->target_yaw_vel =
            Remote_ApplyStickDeadzone(sbus->Channel[0]) /
            kRemoteChannelFullScale * kCommandFullScale;
    snapshot->target_gimbal_pitch_vel =
            Remote_ApplyStickDeadzone(sbus->Channel[1]) /
            kRemoteChannelFullScale * kCommandFullScale;
    snapshot->target_x_vel =
            Remote_ApplyStickDeadzone(sbus->Channel[2]) /
            kRemoteChannelFullScale * kLongitudinalVelocityFullScaleMps;
    snapshot->target_y_vel =
            Remote_ApplyStickDeadzone(sbus->Channel[3]) /
            kRemoteChannelFullScale * kCommandFullScale;
    snapshot->target_gyro_speed =
            Remote_ApplyStickDeadzone(sbus->Channel[4]) /
            kRemoteChannelFullScale * kCommandFullScale;
    snapshot->leg_length_channel = sbus->Channel[5];
    snapshot->step_climb_enable = (sbus->Channel[6] >= 100) ? 1U : 0U;
    snapshot->lqr_enable = (sbus->Channel[7] > 100) ? 1U : 0U;
    snapshot->gyro_mode_enable = (sbus->Channel[8] > 100) ? 1U : 0U;
    snapshot->total_enable = (sbus->Channel[9] >= 100) ? 1U : 0U;
}

void app_Remote_Task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Remote_ControlSnapshot_t remote_snapshot = {};
    (void)pvParameters;
    while (1) {
        /* 本任务周期性取出 UART 环形缓冲中的完整 SBUS 帧；无新帧时保留
         * 上一次有效遥控数据。 */
        bool remote_updated = false;
        while (SBUS_Frame_Parse_From_Buffer(&sbus_ring_buffer, &SBUS_Data) != 0U) {
            remote_updated = true;
        }
        /* 此任务将 SBUS 帧解包成四摇杆、两旋钮、四开关的成品命令快照；
         * CalculateTask 只消费快照并决定这些指令是否接入实际控制。 */
        if (remote_updated && (Remote_ControlQueueHandle != NULL)) {
            Remote_ParseControlSnapshot(&remote_snapshot, &SBUS_Data);
            if (xQueueOverwrite(Remote_ControlQueueHandle, &remote_snapshot) == pdPASS) {
                DataTimeout_MarkRemoteValid();
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
