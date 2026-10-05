#include "Can_Filter.h"
#include "Can_Motor.h"

/**
 * @brief 配置 FDCAN1 的标准帧接收过滤器。
 * @details
 *   Filter 0、1 使用掩码模式，将两个摩擦轮反馈帧分别送入 FIFO0；
 *   Filter 2 使用掩码模式，只将 Pitch 反馈帧送入 FIFO1。
 *   台架诊断期间，未匹配的标准帧也送入 FIFO0，记录其实际 ID；扩展帧和
 *   远程帧仍拒绝。这样即使上位机配置的 MST_ID 与工程不一致，也能定位。
 */
void Can_Filter_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};

  /* 左摩擦轮反馈，精确匹配进入 FIFO0。 */
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = DM_MOTOR_LEFT_FB_ID;
  filter.FilterID2 = 0x7FFU;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  /* 右摩擦轮反馈，精确匹配进入 FIFO0。 */
  filter.FilterIndex = 1;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = DM_MOTOR_RIGHT_FB_ID;
  filter.FilterID2 = 0x7FFU;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  /* Pitch 反馈，精确匹配进入 FIFO1，供云台优先处理。 */
  filter.FilterIndex = 2;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  filter.FilterID1 = DM_MOTOR_PITCH_FB_ID;
  filter.FilterID2 = 0x7FFU;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE);
  (void)HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 5U);
  (void)HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 5U);
}
