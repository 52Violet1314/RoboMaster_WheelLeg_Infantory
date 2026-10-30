#include "app_wheel_kalman_telemetry.hpp"

#include "app_main.hpp"
#include "wheel_kalman_fliter.h"

#include <math.h>
#include <stdlib.h>

void WheelKalmanTelemetry_Print(void) {
    const float avg_x_pos = (Wheel_Kalman_L.x[0] + Wheel_Kalman_R.x[0]) * 0.5f;
    const int pos_milli_m = (int)lroundf(avg_x_pos * 1000.0f);

    /* 与原工程 FireWater 格式完全一致：带符号、三位小数、纯 CSV 数据行。 */
    uart_print("%c%d.%03d\r\n", pos_milli_m < 0 ? '-' : ' ',
               abs(pos_milli_m) / 1000, abs(pos_milli_m) % 1000);
}
