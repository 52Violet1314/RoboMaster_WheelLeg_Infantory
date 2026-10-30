#include "app_wheel_kalman_telemetry.hpp"

#include "app_main.hpp"
#include "wheel_kalman_fliter.h"

void WheelKalmanTelemetry_Print(void) {
    const float avg_x_pos = (Wheel_Kalman_L.x[0] + Wheel_Kalman_R.x[0]) * 0.5f;

    uart_print("%.5f\r\n", avg_x_pos);
}
