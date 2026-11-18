#include "app_chassis_yaw_state.hpp"

#include <math.h>

namespace {
/* 只有遥控已松开、车身自身也基本停转时才捕获当前航向。
 * 进入/退出使用不同阈值，避免 IMU 噪声在门限附近来回切换。 */
constexpr float kYawHoldCommandEnterRadS = 0.08f;
constexpr float kYawHoldCommandExitRadS = 0.12f;
constexpr float kYawHoldRateEnterRadS = 0.15f;
constexpr float kControlPeriodS = 0.001f;
}  // namespace

void ChassisYawState_Update(Classic_Data_t *data, bool yaw_angle_hold_enable) {
    static bool yaw_hold_active = false;

    data->States_Data.d_yaw_err =
            data->IMU_Data.delta_yaw - (-data->Target_Data.Target_yaw_vel);

    if (!yaw_angle_hold_enable) {
        yaw_hold_active = false;
        data->States_Data.yaw_err = 0.0f;
        return;
    }

    const float command_abs = fabsf(data->Target_Data.Target_yaw_vel);
    const float rate_abs = fabsf(data->IMU_Data.delta_yaw);
    if (yaw_hold_active) {
        /* 锁住后仅由新的遥控转向命令解锁。不能用实测角速度解锁，
         * 否则锁角自身的纠偏会触发退出。 */
        if (command_abs > kYawHoldCommandExitRadS) {
            yaw_hold_active = false;
            data->States_Data.yaw_err = 0.0f;
        }
    } else if (command_abs < kYawHoldCommandEnterRadS &&
               rate_abs < kYawHoldRateEnterRadS) {
        yaw_hold_active = true;
        /* 当前航向即新的保持目标；从零误差开始避免切入冲击。 */
        data->States_Data.yaw_err = 0.0f;
    }

    if (!yaw_hold_active) return;

    /* 保持原有 1 ms 控制周期的 yaw 误差积分。 */
    data->States_Data.yaw_err += data->States_Data.d_yaw_err * kControlPeriodS;
    if (data->States_Data.yaw_err > 3.1415926f) {
        data->States_Data.yaw_err = 3.1415926f;
    } else if (data->States_Data.yaw_err < -3.1415926f) {
        data->States_Data.yaw_err = -3.1415926f;
    }
}
