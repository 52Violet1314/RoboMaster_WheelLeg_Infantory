#ifndef __GROUND_CLEARANCE_DETECTION_H__
#define __GROUND_CLEARANCE_DETECTION_H__

#include <stdint.h>

/* 仅控制离地观测器是否在 CalculateTask 中运行。
 * 默认关闭：当前 Fn/离地状态只用于观测，尚未接入正常控制决策。 */
#define GROUND_DETECTION_ENABLE  0

#ifdef __cplusplus
extern "C" {
#endif

#define GD_SIDE_LEFT   0U
#define GD_SIDE_RIGHT  1U

/* 轮地法向力纯观测器（无内部状态）。
 *
 * 坐标约定：竖直向下为正；phi=0 为腿竖直向下，前摆为正。
 * f0_without_spring_n 必须是去除气弹簧内部抵消项后的腿向力；
 * tp_vmc_nm 必须是本轮实际送入 VMC 的虚拟摆力矩（已按状态决定是否叠加重力补偿）。
 * 返回地面对轮子的向上法向支持力幅值(N)。 */
float Ground_Clearance_Detection(float f0_without_spring_n, float tp_vmc_nm, float L0,
                                 float d_L0, float dd_L0, float dd_zm,
                                 float phi, float d_phi, float dd_phi);

/* 带状态检测：一阶低通 + 双向阈值迟滞 + 连续 N 拍确认。
 * f0_command_n 为 LegControl 的最终 F0 命令；函数内部只剔除弹簧补偿。
 * tp_vmc_nm 为本轮实际送入 VMC 的 Tp，不在此处重复计算重力补偿。
 * 每拍每条腿调用一次，返回本拍滤波后的 Fn。 */
float GroundDetection_Update(uint8_t side,
                             float f0_command_n, float tp_vmc_nm, float L0,
                             float d_L0, float dd_L0, float dd_zm,
                             float phi, float d_phi, float dd_phi);

/* 查询接口 */
float   GroundDetection_GetFn(uint8_t side);        /* 滤波后 Fn(N) */
uint8_t GroundDetection_IsOffGround(uint8_t side);  /* 1 = 该腿离地 */
uint8_t GroundDetection_IsAirborne(void);           /* 1 = 双腿都离地 */
void    GroundDetection_Reset(void);

#ifdef __cplusplus
}
#endif

#endif
