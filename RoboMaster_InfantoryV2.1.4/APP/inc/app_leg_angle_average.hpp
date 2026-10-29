#ifndef __APP_LEG_ANGLE_AVERAGE_HPP__
#define __APP_LEG_ANGLE_AVERAGE_HPP__

#include <stdbool.h>
#include <stdint.h>

/*
 * 起身且腿角速度足够小时，连续采样稳定腿角。
 * 采满后结果冻结，直到失去起身状态或再次出现明显摆动才重新采样。
 * 控制不读取本模块的任何结果，仅供台架标定和串口观察。
 */
#define LEG_ANGLE_AVERAGE_SAMPLE_COUNT       1000U  /* CalculateTask 1 ms 周期：约 1 s */
#define LEG_ANGLE_AVERAGE_MAX_RATE_RAD_S     0.050f /* 约 2.9 deg/s */

typedef struct {
    float sum_left_rad;
    float sum_right_rad;
    float average_left_rad;
    float average_right_rad;
    uint32_t sample_count;
    bool ready;
} LegAngleAverage_t;

void LegAngleAverage_Reset(LegAngleAverage_t *average);
void LegAngleAverage_Update(LegAngleAverage_t *average, bool stand_up,
                            float theta_left_rad, float theta_right_rad,
                            float dtheta_left_rad_s, float dtheta_right_rad_s);

/* 结果为 L/R 各自平均值及两侧平均值，单位均为 rad。 */
float LegAngleAverage_GetLeftRad(const LegAngleAverage_t *average);
float LegAngleAverage_GetRightRad(const LegAngleAverage_t *average);
float LegAngleAverage_GetMeanRad(const LegAngleAverage_t *average);

#endif
