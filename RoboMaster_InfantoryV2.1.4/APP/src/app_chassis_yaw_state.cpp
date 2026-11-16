#include "app_chassis_yaw_state.hpp"

void ChassisYawState_Update(Classic_Data_t *data, bool yaw_angle_hold_enable) {
    data->States_Data.d_yaw_err =
            data->IMU_Data.delta_yaw - (-data->Target_Data.Target_yaw_vel);

    if (!yaw_angle_hold_enable) {
        data->States_Data.yaw_err = 0.0f;
        return;
    }

    /* 保持原有 1 ms 控制周期的 yaw 误差积分，避免封装改变控制律。 */
    data->States_Data.yaw_err += data->States_Data.d_yaw_err / 1000.0f;
    if (data->States_Data.yaw_err > 3.1415926f) {
        data->States_Data.yaw_err = 3.1415926f;
    } else if (data->States_Data.yaw_err < -3.1415926f) {
        data->States_Data.yaw_err = -3.1415926f;
    }
}
