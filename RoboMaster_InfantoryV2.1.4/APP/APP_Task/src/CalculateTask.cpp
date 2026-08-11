#include "CalculateTask.hpp"
#include "ControlTask.hpp"
#include "DMIMU.h"
#include "FreeRTOS.h"
#include "PID.h"
#include "POWER.h"
#include "Power.h"
#include "SBUS.h"
#include "WS2812.h"
#include "app_K_value.hpp"
#include "app_Task.hpp"
#include "app_Data_Task.hpp"
#include "app_ins_cal.h"
#include "app_motor.h"
#include "arm_math.h"
#include "bsp_UART.h"
#include "event_groups.h"
#include "fdcan.h"
#include "main.h"
#include "math.h"
#include "queue.h"
#include "stm32h7xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <cstdint>
#include <cstdio>
#include <stdint.h>
#include <type_traits>
#include "wheel_kalman_fliter.h"
#include "Ground_clearance_detection.h"

// 定义参考值（根据实际需求调整）
float d_X_ref = 0.0f; // 期望速度
float theta_ref = 0.0f; // 期望整体角度
float d_yaw_ref = 0.0f; // 期望整体角速度
float yaw_ref = 0.0f; // 期望整体角速度
float d_theta_ref = 0.0f; // 期望整体角速度
float theta_L_ref = 0.0f; // 期望左腿角度
float d_theta_L_ref = 0.0f; // 期望左腿角速度
float theta_R_ref = 0.0f; // 期望右腿角度
float d_theta_R_ref = 0.0f; // 期望右腿角速度

Float_PID_Typedef LEG_PD_L_PID;
Float_PID_Typedef LEG_PD_R_PID;
Float_PID_Typedef LEG_PHI_PD_L_PID;
Float_PID_Typedef LEG_PHI_PD_R_PID;
Float_PID_Typedef LEG_DIFF_PID;

bool StandUp_Flag = true;

uint8_t ControlState = 0;

#define a 0.2f // 一阶低通滤波

int task_time = 0;
int last_task_time = 0;
int Calculate_time;

float Target_Leg_Long = 0.150f;

typedef enum
{
  ChangingToHighLegLength,
  ChangingToLowLegLength,
  ApproachingLegLength,
  StableLegLength,
} LegLengthState_t;

LegLengthState_t Left_LegLengthState = StableLegLength;
LegLengthState_t Right_LegLengthState = StableLegLength;

uint8_t LeftLegStableFlag = 0;
uint8_t RightLegStableFlag = 0;



void VMC_Calculate(Classic_Data_t *data, float F0L, float F0R, float TpL,
                   float TpR, float dt);

float angle_to_rad(float degrees);

void LEG_Data_Print(Classic_Data_t *pClassicData);

void Motor_Data_Print(Classic_Data_t *pClassicData);

void Control_Data_Print(Classic_Data_t *pClassicData);

void IMU_Data_Print(Classic_Data_t *pClassicData);

void State_Data_Print(Classic_Data_t *pClassicData);

void Classic_Data_Update(Classic_Data_t *pClassicData);

void IMU_Temp_Control(float Target_Temp);

void LQR_K_Calculate350(Classic_Data_t *pClassicData);
void LQR_K_Calculate250(Classic_Data_t *pClassicData);
void LQR_K_Calculate150(Classic_Data_t *pClassicData);

void Controler_Limit(Classic_Data_t *pClassicData);

void CaculateTask(void *pvParameters) {
    float Pos_INT = 0.0f;
    float leg_l_int_max = 0.0f;
    float leg_l_res_max = 0.0f;
    float leg_r_int_max = 0.0f;
    float leg_r_res_max = 0.0f;
    float leg_l_err = 0.0f;
    float leg_r_err = 0.0f;
    static float Max_F0 = 50.0f;
    static float Max_Tp = 1.8f;
    uint16_t StandUp_Count = 0;
    Classic_Data_t *pClassicData = (Classic_Data_t *) pvParameters;
    Float_PID_Init(&LEG_PD_L_PID, 400.0f, 1.5f, 2.0f);
    Float_PID_Init(&LEG_PD_R_PID, 400.0f, 1.5f, 2.0f);
    Float_PID_Init(&LEG_PHI_PD_L_PID, 15.0f, 0.0f, 3.0f);
    Float_PID_Init(&LEG_PHI_PD_R_PID, 15.0f, 0.0f, 3.0f);
    Float_PID_Init(&LEG_DIFF_PID, 100.0f, 1.2f, 1.0f);

    // --- DM 电机初始化（从 Data Task 移入） ---
    DM_Motor_Init(&DM_8009P1, 0x051, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P2, 0x052, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P3, 0x053, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P4, 0x054, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_3519L, 0x055, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_3519R, 0x056, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    
    WheelKalman_Init(&Wheel_Kalman_L, 0.001f, 0.0f, 0.0f);
    WheelKalman_Init(&Wheel_Kalman_R, 0.001f, 0.0f, 0.0f);

    // 串口 DMA 接收初始化（从 Data Task 移入）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE * 2);

    uart_print("CaculateTask start\r\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_cal_tick = 0;  // 使用 htim5 微秒计数器跟踪真实 dt
    while (1) {
        // ====== 原 Data Task：等待传感器数据就绪（5ms 超时保护） ======
        EventBits_t uxBits = xEventGroupWaitBits(
            DataGroup, MOTOR_Data_READY_BIT | IMU_DATA_READY_BIT,
            pdTRUE, pdTRUE, pdMS_TO_TICKS(10));

        if ((uxBits & (MOTOR_Data_READY_BIT | IMU_DATA_READY_BIT))
            != (MOTOR_Data_READY_BIT | IMU_DATA_READY_BIT)) {
            /* 节流打印：每 500ms 最多输出一次，避免串口洪水拖慢调度 */
            static TickType_t xLastTimeoutPrint = 0;
            extern uint32_t fdcan_tx_fail_count;
            TickType_t xNow = xTaskGetTickCount();
            if ((xNow - xLastTimeoutPrint) > pdMS_TO_TICKS(500)) {
                uart_print("Cal: sensor timeout bits=0x%02X TX_fail=%lu\r\n",
                           (unsigned)uxBits, fdcan_tx_fail_count);
                xLastTimeoutPrint = xNow;
            }
            xEventGroupSetBits(ControlEventGroup, Calculate_OK_BIT);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
            continue;
        }

        // ====== 计算本周期真实时间步长 (s) ======
        uint32_t now_tick = __HAL_TIM_GET_COUNTER(&htim5);
        float cal_dt = (float)(now_tick - last_cal_tick) * 1e-6f;
        if (cal_dt <= 0.0f || cal_dt > 0.1f) cal_dt = 0.001f;  // 1ms 默认值
        last_cal_tick = now_tick;

        // ====== 原 Data Task：读取电机数据 ======
        pClassicData->Motor_Data.L1_Motor_POS =
                (DM_8009P1.data.position_rad / 3.50f * 3.14f) + 3.14f;
        pClassicData->Motor_Data.L2_Motor_POS =
                (DM_8009P2.data.position_rad / 3.50f * 3.14f) + 6.28f;
        pClassicData->Motor_Data.R1_Motor_POS =
                -(DM_8009P4.data.position_rad / 3.50f * 3.14f) - 3.14f;
        pClassicData->Motor_Data.R2_Motor_POS =
                -(DM_8009P3.data.position_rad / 3.50f * 3.14f);
        pClassicData->Motor_Data.L1_Motor_VEL = DM_8009P1.data.velocity_rad_s;
        pClassicData->Motor_Data.L2_Motor_VEL = DM_8009P2.data.velocity_rad_s;
        pClassicData->Motor_Data.R1_Motor_VEL = DM_8009P4.data.velocity_rad_s;
        pClassicData->Motor_Data.R2_Motor_VEL = DM_8009P3.data.velocity_rad_s;
        pClassicData->Motor_Data.Left_Wheel_Motor_VEL =
                -(DM_3519L.data.velocity_rad_s / 15.8f * 19.2f + 0.048f);
        pClassicData->Motor_Data.Right_Wheel_Motor_VEL =
                (DM_3519R.data.velocity_rad_s / 15.8f * 19.2f + 0.048f);

        // ====== 原 Data Task：计算状态变量 ======
        // pClassicData->States_Data.d_X =
        //         (pClassicData->Motor_Data.Left_Wheel_Motor_VEL * -0.05f);
        pClassicData->States_Data.d_X = ((Wheel_Kalman_L.x[1] + Wheel_Kalman_R.x[1]) / 2.0f);
        Pos_INT += pClassicData->Target_Data.Target_X_vel / 1000.0f;
                //  pClassicData->Motor_Data.Right_Wheel_Motor_VEL * -0.05f) / 2.0f;
        pClassicData->States_Data.X_pos = ((Wheel_Kalman_L.x[0] + Wheel_Kalman_R.x[0]) / 2.0f) - Pos_INT;
        // if (pClassicData->States_Data.X_pos > -5.0f)
        //   pClassicData->States_Data.X_pos = -5.0f;
        // if (pClassicData->States_Data.X_pos < -15.0f)
        //   pClassicData->States_Data.X_pos = -15.0f;
        pClassicData->States_Data.d_x_err =
                pClassicData->States_Data.d_X - pClassicData->Target_Data.Target_X_vel;
        pClassicData->States_Data.d_yaw_err =
                pClassicData->Target_Data.Target_yaw_vel + pClassicData->IMU_Data.delta_yaw;
        pClassicData->States_Data.yaw_err +=
                (pClassicData->States_Data.d_yaw_err) / 1000.f;
        if (pClassicData->States_Data.yaw_err > 3.14f)
          pClassicData->States_Data.yaw_err = 3.14f;
        if (pClassicData->States_Data.yaw_err < -3.14f)
          pClassicData->States_Data.yaw_err = -3.14f;
        pClassicData->States_Data.theta_L =
                +1.5708f - pClassicData->Leg_Data.phi_0_L - pClassicData->IMU_Data.pitch;
        pClassicData->States_Data.d_theta_L = -pClassicData->Leg_Data.delta_phi_0_L -
                                              pClassicData->IMU_Data.delta_pitch;
        pClassicData->States_Data.theta_R =
                +1.5708f - pClassicData->Leg_Data.phi_0_R - pClassicData->IMU_Data.pitch;
        pClassicData->States_Data.d_theta_R = -pClassicData->Leg_Data.delta_phi_0_R -
                                              pClassicData->IMU_Data.delta_pitch;
        pClassicData->States_Data.theta = pClassicData->IMU_Data.pitch;
        pClassicData->States_Data.d_theta = pClassicData->IMU_Data.delta_pitch;

        /* theta_L / theta_R 二阶导（角加速度）：后向差分 + 一阶低通，使用定时器 dt */
        {
            float inv_dt = 1.0f / cal_dt;
            float raw_dd_theta_L = (pClassicData->States_Data.d_theta_L
                                  - pClassicData->States_Data.d_last_d_theta_L) * inv_dt;
            pClassicData->States_Data.dd_theta_L =
                    a * raw_dd_theta_L + (1 - a) * pClassicData->States_Data.dd_theta_L;
            pClassicData->States_Data.d_last_d_theta_L = pClassicData->States_Data.d_theta_L;

            float raw_dd_theta_R = (pClassicData->States_Data.d_theta_R
                                  - pClassicData->States_Data.d_last_d_theta_R) * inv_dt;
            pClassicData->States_Data.dd_theta_R =
                    a * raw_dd_theta_R + (1 - a) * pClassicData->States_Data.dd_theta_R;
            pClassicData->States_Data.d_last_d_theta_R = pClassicData->States_Data.d_theta_R;
        }
        
        WheelKalman_Update(&Wheel_Kalman_L, pClassicData->IMU_Data.accel_h, cal_dt, -pClassicData->States_Data.theta_L, -pClassicData->States_Data.d_theta_L, pClassicData->Leg_Data.L0_L, pClassicData->Leg_Data.d_L0_L, pClassicData->IMU_Data.delta_pitch,pClassicData->Motor_Data.Left_Wheel_Motor_VEL);
        WheelKalman_Update(&Wheel_Kalman_R, pClassicData->IMU_Data.accel_h, cal_dt, -pClassicData->States_Data.theta_R, -pClassicData->States_Data.d_theta_R, pClassicData->Leg_Data.L0_R, pClassicData->Leg_Data.d_L0_R, pClassicData->IMU_Data.delta_pitch,pClassicData->Motor_Data.Right_Wheel_Motor_VEL);
        
        // WheelKalman_Print(&Wheel_Kalman_L, 'L');
        // WheelKalman_Print(&Wheel_Kalman_R, 'R');
        float Fn_Left = Ground_Clearance_Detection(pClassicData->Contronller_Data.F0_L,pClassicData->Contronller_Data.Tp_L, pClassicData->Leg_Data.L0_L,pClassicData->Leg_Data.d_L0_L,pClassicData->Leg_Data.dd_L0_L,pClassicData->IMU_Data.accel_v,pClassicData->States_Data.theta_L,pClassicData->States_Data.d_theta_L,pClassicData->States_Data.dd_theta_L);
        float Fn_Right = Ground_Clearance_Detection(pClassicData->Contronller_Data.F0_R,pClassicData->Contronller_Data.Tp_R, pClassicData->Leg_Data.L0_R,pClassicData->Leg_Data.d_L0_R,pClassicData->Leg_Data.dd_L0_R,pClassicData->IMU_Data.accel_v,pClassicData->States_Data.theta_R,pClassicData->States_Data.d_theta_R,pClassicData->States_Data.dd_theta_R);
        // ====== LQR 平衡控制 / 腿长目标选择 ======
        if (SBUS_Data.Channel[7] < -100) {
            pClassicData->Contronller_Data.Tp_L = 0.0f;
            pClassicData->Contronller_Data.Tp_R = 0.0f;
            pClassicData->Contronller_Data.Fw_L = 0.0f;
            pClassicData->Contronller_Data.Fw_R = 0.0f;
            Target_Leg_Long = 0.150f;
        }
        else if (SBUS_Data.Channel[7] > 100) {
            if (SBUS_Data.Channel[8] > 100) {
                Target_Leg_Long = 0.35f;
                LQR_K_Calculate350(pClassicData);
            }
            else if (SBUS_Data.Channel[8] < -100) {
                Target_Leg_Long = 0.15f;
                LQR_K_Calculate150(pClassicData);
            }
            else {
                Target_Leg_Long = 0.25f;
                LQR_K_Calculate250(pClassicData);
            }
        }
        leg_l_err = pClassicData->Leg_Data.L0_L - Target_Leg_Long;
        leg_r_err = pClassicData->Leg_Data.L0_R - Target_Leg_Long;
        if(leg_l_err > 0.050f)
        {
            Left_LegLengthState = ChangingToHighLegLength;
        }
        else if(leg_l_err < -0.050f)
        {
            Left_LegLengthState = ChangingToLowLegLength;
        }
        if(leg_l_err < 0.030f && leg_l_err > -0.030f)
        {
            if(leg_l_err < 0.010f && leg_l_err > -0.010f)
            {
                if(LeftLegStableFlag >= 10)
                {
                    Left_LegLengthState = StableLegLength;
                }
                else LeftLegStableFlag ++;
            }
            else
            {
                Left_LegLengthState = ApproachingLegLength;
            }
        }
        switch(Left_LegLengthState)
        {
            case StableLegLength:
                leg_l_int_max = 5.0f;
                leg_l_res_max = 12.0f;
                LEG_PD_L_PID.Kp = 1200.0f;
                LEG_PD_L_PID.Ki = 1.5f;
                break;
            case ChangingToHighLegLength:
                leg_l_int_max = 5.0f;
                leg_l_res_max = 17.5f;
                LEG_PD_L_PID.Kp = 1500.0f;
                LEG_PD_L_PID.Ki = 2.0f;
                break;
            case ChangingToLowLegLength:
                leg_l_int_max = 1.0f;
                leg_l_res_max = 2.0f;
                LEG_PD_L_PID.Kp = 200.0f;
                LEG_PD_L_PID.Ki = 0.5f;
                break;
            case ApproachingLegLength:
                leg_l_int_max = 5.0f;
                leg_l_res_max = 15.5f;
                LEG_PD_L_PID.Kp = 1200.0f;
                LEG_PD_L_PID.Ki = 2.0f;
                break;
        }

        if(leg_r_err > 0.050f)
        {
            Right_LegLengthState = ChangingToHighLegLength;
        }
        else if(leg_r_err < -0.050f)
        {
            Right_LegLengthState = ChangingToLowLegLength;
        }
        if(leg_r_err < 0.030f && leg_r_err > -0.030f)
        {
            if(leg_r_err < 0.010f && leg_r_err > -0.010f)
            {
                if(RightLegStableFlag >= 10)
                {
                    Right_LegLengthState = StableLegLength;
                }
                else RightLegStableFlag ++;
            }
            else
            {
                Right_LegLengthState = ApproachingLegLength;
            }
        }
        switch(Right_LegLengthState)
        {
            case StableLegLength:
                leg_r_int_max = 5.0f;
                leg_r_res_max = 12.0f;
                LEG_PD_R_PID.Kp = 1000.0f;
                LEG_PD_R_PID.Ki = 1.5f;
                break;
            case ChangingToHighLegLength:
                leg_r_int_max = 5.0f;
                leg_r_res_max = 17.5f;
                LEG_PD_R_PID.Kp = 1500.0f;
                LEG_PD_R_PID.Ki = 2.0f;
                break;
            case ChangingToLowLegLength:
                leg_r_int_max = 1.0f;
                leg_r_res_max = 2.0f;
                LEG_PD_R_PID.Kp = 200.0f;
                LEG_PD_R_PID.Ki = 0.5f;
                break;
            case ApproachingLegLength:
                leg_r_int_max = 5.0f;
                leg_r_res_max = 15.5f;
                LEG_PD_R_PID.Kp = 1200.0f;
                LEG_PD_R_PID.Ki = 2.0f;
                break;
        }

        PID_Max_Float(&LEG_PD_L_PID, Target_Leg_Long, pClassicData->Leg_Data.L0_L, leg_l_int_max, leg_l_res_max);
        PID_Max_Float(&LEG_PD_R_PID, Target_Leg_Long, pClassicData->Leg_Data.L0_R, leg_r_int_max, leg_r_res_max);

        // ====== 左右腿长差PID纠偏 ======
        float leg_length_diff = pClassicData->Leg_Data.L0_L - pClassicData->Leg_Data.L0_R;
        PID_Max_Float(&LEG_DIFF_PID, 0.0f, leg_length_diff, 5.0f, 10.0f);

        // ====== 腿长力输出（重力补偿 + 差动纠偏）======
        pClassicData->Contronller_Data.F0_L = LEG_PD_L_PID.Res + 25.5f + LEG_DIFF_PID.Res * 0.5f;
        pClassicData->Contronller_Data.F0_R = LEG_PD_R_PID.Res + 25.5f - LEG_DIFF_PID.Res * 0.5f;
       
        if(SBUS_Data.Channel[9] < 100) 
        {
            StandUp_Flag = true;
            // uart_print("StandUp_Flag = true\n");
        }
        else if(pClassicData->States_Data.theta_L < 0.2f && pClassicData->States_Data.theta_L > -0.2f)
        {   
            if(StandUp_Count >= 500)
            {
                StandUp_Flag = false;
            // uart_print("StandUp_Flag = false\n");
            }
            else StandUp_Count ++;
        }

        // ====== 输入限幅：VMC 之前，F0/Tp 必须在此限幅（Fw 不经过 VMC） ======
        if(StandUp_Flag == true)
        {
        Max_F0 = 30.0f;
        Max_Tp = 2.0f;
        }
        else  
        {
        Max_F0 = 60.0f;
        Max_Tp = 4.0f;
        };

        //F0 限幅：±50.0 N（腿长力，PID输出上限10 + 偏移10）
        if (pClassicData->Contronller_Data.F0_L > Max_F0)
            pClassicData->Contronller_Data.F0_L = Max_F0;
        if (pClassicData->Contronller_Data.F0_L < -Max_F0)
            pClassicData->Contronller_Data.F0_L = -Max_F0;
        if (pClassicData->Contronller_Data.F0_R > Max_F0)
            pClassicData->Contronller_Data.F0_R = Max_F0;
        if (pClassicData->Contronller_Data.F0_R < -Max_F0)
            pClassicData->Contronller_Data.F0_R = -Max_F0;

        if (pClassicData->Contronller_Data.Tp_L > Max_Tp)
            pClassicData->Contronller_Data.Tp_L = Max_Tp;
        if (pClassicData->Contronller_Data.Tp_L < -Max_Tp)
            pClassicData->Contronller_Data.Tp_L = -Max_Tp;
        if (pClassicData->Contronller_Data.Tp_R > Max_Tp)
            pClassicData->Contronller_Data.Tp_R = Max_Tp;
        if (pClassicData->Contronller_Data.Tp_R < -Max_Tp)
            pClassicData->Contronller_Data.Tp_R = -Max_Tp;
        // ====== VMC 力矩分配 ======
        VMC_Calculate(pClassicData, pClassicData->Contronller_Data.F0_L,
                      pClassicData->Contronller_Data.F0_R,
                      pClassicData->Contronller_Data.Tp_L,
                      pClassicData->Contronller_Data.Tp_R, cal_dt);

        // ====== 输出限幅：VMC 之后，T1/T2 限幅 ======
        Controler_Limit(pClassicData);

        /* 打印节流：两次打印至少间隔 50ms，避免 DMA 串口跟不上导致 CalculateTask 阻塞超时 */
        {
            static TickType_t xLastPrintTick = 0;
            TickType_t xNow = xTaskGetTickCount();
            if(SBUS_Data.Channel[7] > 100)
            {
               if ((xNow - xLastPrintTick) > pdMS_TO_TICKS(50)) {
                    State_Data_Print(pClassicData);
                xLastPrintTick = xNow;
            }
            }

        }
        // uart_print("%d,%d,%d,%d,%d,%d,%d\r\n",(int)(Target_Leg_Long * 1000),(int)(pClassicData->Leg_Data.L0_L * 1000),(int)(leg_l_err * 1000),(int)(pClassicData->Contronller_Data.F0_L * 1000),(int)(pClassicData->Leg_Data.L0_R * 1000),(int)(leg_r_err * 1000),(int)(pClassicData->Contronller_Data.F0_R * 1000));
        xEventGroupSetBits(ControlEventGroup, Calculate_OK_BIT);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

void VMC_Calculate(Classic_Data_t *data, float F0L, float F0R, float TpL,
                   float TpR, float dt) {
    float inv_dt = 1.0f / dt;

    // 机械臂连杆长度定义
    float l1 = 0.210f;
    float l2 = 0.250f;
    float l3 = 0.250f;
    float l4 = 0.210f;

    // 从结构体中获取输入参数
    float phi1_L = data->Motor_Data.L1_Motor_POS; // L1_Motor_POS 对应左侧 phi1
    float phi4_L = data->Motor_Data.L2_Motor_POS; // L2_Motor_POS 对应左侧 phi4

    float phi1_R = data->Motor_Data.R1_Motor_POS; // R1_Motor_POS 对应右侧 phi1
    float phi4_R = data->Motor_Data.R2_Motor_POS; // R2_Motor_POS 对应右侧 phi4

    // **************************
    // 左侧力矩计算
    // **************************
    // 计算各点坐标
    float YD_L = l4 * arm_sin_f32(phi4_L);
    float YB_L = l1 * arm_sin_f32(phi1_L);
    float XD_L = l4 * arm_cos_f32(phi4_L);
    float XB_L = l1 * arm_cos_f32(phi1_L);

    // 计算 BD 长度
    float XD_XB_L = XD_L - XB_L;
    float YD_YB_L = YD_L - YB_L;
    float temp_L = XD_XB_L * XD_XB_L + YD_YB_L * YD_YB_L;
    float lBD_L;
    arm_sqrt_f32(temp_L, &lBD_L);

    // 计算 phi2
    float A0_L = 2 * l2 * XD_XB_L;
    float B0_L = 2 * l2 * YD_YB_L;
    float C0_L = l2 * l2 + lBD_L * lBD_L - l3 * l3;
    float temp2_L = A0_L * A0_L + B0_L * B0_L - C0_L * C0_L;
    float sqrt_val_L;
    arm_sqrt_f32(temp2_L, &sqrt_val_L);
    float phi2_L = 2 * atan2f((B0_L + sqrt_val_L), (A0_L + C0_L));

    // 计算 phi3
    float YB_YD_L = YB_L - YD_L;
    float XB_XD_L = XB_L - XD_L;
    float phi3_L = atan2f(YB_YD_L + l2 * arm_sin_f32(phi2_L),
                          XB_XD_L + l2 * arm_cos_f32(phi2_L));

    // 计算 XC, YC
    float XC_L = l1 * arm_cos_f32(phi1_L) + l2 * arm_cos_f32(phi2_L);
    float YC_L = l1 * arm_sin_f32(phi1_L) + l2 * arm_sin_f32(phi2_L);

    // 计算 L0, phi0
    float XC_l5_2_L = XC_L;
    float temp3_L = XC_l5_2_L * XC_l5_2_L + YC_L * YC_L;
    float L0_L;
    arm_sqrt_f32(temp3_L, &L0_L);
    float phi0_L = atan2f(YC_L, XC_l5_2_L);
    data->Leg_Data.phi_0_L = phi0_L;
    data->Leg_Data.L0_L = L0_L;

    /* L0_L 一阶导 / 二阶导：后向差分 + 一阶低通滤波，使用真实 dt */
    {
        static float last_L0_L = 0.0f;
        float raw_d_L0_L = (L0_L - last_L0_L) * inv_dt;          // m/s
        data->Leg_Data.d_L0_L =
                a * raw_d_L0_L + (1 - a) * data->Leg_Data.d_L0_L;

        float raw_dd_L0_L = (data->Leg_Data.d_L0_L - data->Leg_Data.d_last_L0_L) * inv_dt;
        data->Leg_Data.dd_L0_L =
                a * raw_dd_L0_L + (1 - a) * data->Leg_Data.dd_L0_L;
        data->Leg_Data.d_last_L0_L = data->Leg_Data.d_L0_L;
        last_L0_L = L0_L;
    }

    static float last_phi_0_L = 0.0f;
    float raw_delta_phi_0_L = (phi0_L - last_phi_0_L) * 1000.0f;
    data->Leg_Data.delta_phi_0_L =
            a * raw_delta_phi_0_L + (1 - a) * data->Leg_Data.delta_phi_0_L;
    last_phi_0_L = phi0_L;

    // 计算雅可比矩阵元素
    float sin_phi0_phi3_L = arm_sin_f32(phi0_L - phi3_L);
    float sin_phi1_phi2_L = arm_sin_f32(phi1_L - phi2_L);
    float sin_phi3_phi2_L = arm_sin_f32(phi3_L - phi2_L);
    float cos_phi0_phi3_L = arm_cos_f32(phi0_L - phi3_L);
    float sin_phi0_phi2_L = arm_sin_f32(phi0_L - phi2_L);
    float sin_phi3_phi4_L = arm_sin_f32(phi3_L - phi4_L);
    float cos_phi0_phi2_L = arm_cos_f32(phi0_L - phi2_L);

    float j11_L = (l1 * sin_phi0_phi3_L * sin_phi1_phi2_L) / sin_phi3_phi2_L;
    float j12_L =
            (l1 * cos_phi0_phi3_L * sin_phi1_phi2_L) / (L0_L * sin_phi3_phi2_L);
    float j21_L = (l4 * sin_phi0_phi2_L * sin_phi3_phi4_L) / sin_phi3_phi2_L;
    float j22_L =
            (l4 * cos_phi0_phi2_L * sin_phi3_phi4_L) / (L0_L * sin_phi3_phi2_L);

    // 计算力矩
    float F_L[2] = {F0L, TpL};
    float T_L[2];
    T_L[0] = j11_L * F_L[0] + j12_L * F_L[1];
    T_L[1] = j21_L * F_L[0] + j22_L * F_L[1];

    // **************************
    // 右侧力矩计算
    // **************************
    // 计算各点坐标
    float YD_R = l4 * arm_sin_f32(phi4_R);
    float YB_R = l1 * arm_sin_f32(phi1_R);
    float XD_R = l4 * arm_cos_f32(phi4_R);
    float XB_R = l1 * arm_cos_f32(phi1_R);

    // 计算 BD 长度
    float XD_XB_R = XD_R - XB_R;
    float YD_YB_R = YD_R - YB_R;
    float temp_R = XD_XB_R * XD_XB_R + YD_YB_R * YD_YB_R;
    float lBD_R;
    arm_sqrt_f32(temp_R, &lBD_R);

    // 计算 phi2
    float A0_R = 2 * l2 * XD_XB_R;
    float B0_R = 2 * l2 * YD_YB_R;
    float C0_R = l2 * l2 + lBD_R * lBD_R - l3 * l3;
    float temp2_R = A0_R * A0_R + B0_R * B0_R - C0_R * C0_R;
    float sqrt_val_R;
    arm_sqrt_f32(temp2_R, &sqrt_val_R);
    float phi2_R = 2 * atan2f((B0_R + sqrt_val_R), (A0_R + C0_R));

    // 计算 phi3
    float YB_YD_R = YB_R - YD_R;
    float XB_XD_R = XB_R - XD_R;
    float phi3_R = atan2f(YB_YD_R + l2 * arm_sin_f32(phi2_R),
                          XB_XD_R + l2 * arm_cos_f32(phi2_R));

    // 计算 XC, YC
    float XC_R = l1 * arm_cos_f32(phi1_R) + l2 * arm_cos_f32(phi2_R);
    float YC_R = l1 * arm_sin_f32(phi1_R) + l2 * arm_sin_f32(phi2_R);

    // 计算 L0, phi0
    float XC_l5_2_R = XC_R;
    float temp3_R = XC_l5_2_R * XC_l5_2_R + YC_R * YC_R;
    float L0_R;
    arm_sqrt_f32(temp3_R, &L0_R);
    float phi0_R = atan2f(YC_R, XC_l5_2_R);
    data->Leg_Data.phi_0_R = phi0_R;
    data->Leg_Data.L0_R = L0_R;

    /* L0_R 一阶导 / 二阶导：后向差分 + 一阶低通滤波，使用真实 dt */
    {
        static float last_L0_R = 0.0f;
        float raw_d_L0_R = (L0_R - last_L0_R) * inv_dt;          // m/s
        data->Leg_Data.d_L0_R =
                a * raw_d_L0_R + (1 - a) * data->Leg_Data.d_L0_R;

        float raw_dd_L0_R = (data->Leg_Data.d_L0_R - data->Leg_Data.d_last_L0_R) * inv_dt;
        data->Leg_Data.dd_L0_R =
                a * raw_dd_L0_R + (1 - a) * data->Leg_Data.dd_L0_R;
        data->Leg_Data.d_last_L0_R = data->Leg_Data.d_L0_R;
        last_L0_R = L0_R;
    }

    static float last_phi_0_R = 0.0f;
    float raw_delta_phi_0_R = (phi0_R - last_phi_0_R) * 1000.0f;
    data->Leg_Data.delta_phi_0_R =
            a * raw_delta_phi_0_R + (1 - a) * data->Leg_Data.delta_phi_0_R;
    last_phi_0_R = phi0_R;

    // 计算雅可比矩阵元素
    float sin_phi0_phi3_R = arm_sin_f32(phi0_R - phi3_R);
    float sin_phi1_phi2_R = arm_sin_f32(phi1_R - phi2_R);
    float sin_phi3_phi2_R = arm_sin_f32(phi3_R - phi2_R);
    float cos_phi0_phi3_R = arm_cos_f32(phi0_R - phi3_R);
    float sin_phi0_phi2_R = arm_sin_f32(phi0_R - phi2_R);
    float sin_phi3_phi4_R = arm_sin_f32(phi3_R - phi4_R);
    float cos_phi0_phi2_R = arm_cos_f32(phi0_R - phi2_R);

    float j11_R = (l1 * sin_phi0_phi3_R * sin_phi1_phi2_R) / sin_phi3_phi2_R;
    float j12_R =
            (l1 * cos_phi0_phi3_R * sin_phi1_phi2_R) / (L0_R * sin_phi3_phi2_R);
    float j21_R = (l4 * sin_phi0_phi2_R * sin_phi3_phi4_R) / sin_phi3_phi2_R;
    float j22_R =
            (l4 * cos_phi0_phi2_R * sin_phi3_phi4_R) / (L0_R * sin_phi3_phi2_R);

    // 计算力矩
    float F_R[2] = {F0R, TpR};
    float T_R[2];
    T_R[0] = j11_R * F_R[0] + j12_R * F_R[1];
    T_R[1] = j21_R * F_R[0] + j22_R * F_R[1];

    // 将结果保存到结构体中
    data->Contronller_Data.T1_L = T_L[0]; // 左侧 T1
    data->Contronller_Data.T2_L = T_L[1]; // 左侧 T2
    data->Contronller_Data.T1_R = T_R[0]; // 右侧 T1
    data->Contronller_Data.T2_R = T_R[1]; // 右侧 T2

    // 保存输入的Tp值
    data->Contronller_Data.Tp_L = TpL; // 左侧 Tp
    data->Contronller_Data.Tp_R = TpR; // 右侧 Tp
}

/**
 * @brief 将角度（度）转换为弧度
 * @param degrees 输入角度值（度）
 * @return float 转换后的弧度值
 */
float angle_to_rad(float degrees) { return degrees * (PI / 180.0f); }

void LEG_Data_Print(Classic_Data_t *pClassicData) {
    uart_print("l0_L:%d.%3d, l0_R:%d.%3d,phi0_L:%d.%3d,phi0_R:%d.%3d",
               (int) (pClassicData->Leg_Data.L0_L * 1000) / 1000,
               (int) (pClassicData->Leg_Data.L0_L * 1000) % 1000,
               (int) (pClassicData->Leg_Data.L0_R * 1000) / 1000,
               (int) (pClassicData->Leg_Data.L0_R * 1000) % 1000,
               (int) (pClassicData->Leg_Data.phi_0_L * 1000) / 1000,
               (int) (pClassicData->Leg_Data.phi_0_L * 1000) % 1000,
               (int) (pClassicData->Leg_Data.phi_0_R * 1000) / 1000,
               (int) (pClassicData->Leg_Data.phi_0_R * 1000) % 1000);
}



void Motor_Data_Print(Classic_Data_t *pClassicData) {
    uart_print(
        "phi1_L:%d.%3d phi4_L:%d.%3d phi1_R:%d.%3d phi4_R:%d.%3d d_phi1L:%d.%3d "
        "d_phi4L:%d.%3d d_phi1_R:%d.%3d d_phi4R:%d.%3d L_wheel_VEL:%d.%3d "
        "R_wheel_VEL:%d.%3d\r\n ",
        (int) (pClassicData->Motor_Data.L1_Motor_POS * 1000) / 1000,
        (int) (pClassicData->Motor_Data.L1_Motor_POS * 1000) % 1000,
        (int) (pClassicData->Motor_Data.L2_Motor_POS * 1000) / 1000,
        (int) (pClassicData->Motor_Data.L2_Motor_POS * 1000) % 1000,
        (int) (pClassicData->Motor_Data.R1_Motor_POS * 1000) / 1000,
        (int) (pClassicData->Motor_Data.R1_Motor_POS * 1000) % 1000,
        (int) (pClassicData->Motor_Data.R2_Motor_POS * 1000) / 1000,
        (int) (pClassicData->Motor_Data.R2_Motor_POS * 1000) % 1000,
        (int) (pClassicData->Motor_Data.L1_Motor_VEL * 1000) / 1000,
        (int) (pClassicData->Motor_Data.L1_Motor_VEL * 1000) % 1000,
        (int) (pClassicData->Motor_Data.L2_Motor_VEL * 1000) / 1000,
        (int) (pClassicData->Motor_Data.L2_Motor_VEL * 1000) % 1000,
        (int) (pClassicData->Motor_Data.R1_Motor_VEL * 1000) / 1000,
        (int) (pClassicData->Motor_Data.R1_Motor_VEL * 1000) % 1000,
        (int) (pClassicData->Motor_Data.R2_Motor_VEL * 1000) / 1000,
        (int) (pClassicData->Motor_Data.R2_Motor_VEL * 1000) % 1000,
        (int) (pClassicData->Motor_Data.Left_Wheel_Motor_VEL * 1000) / 1000,
        (int) (pClassicData->Motor_Data.Left_Wheel_Motor_VEL * 1000) % 1000,
        (int) (pClassicData->Motor_Data.Right_Wheel_Motor_VEL * 1000) / 1000,
        (int) (pClassicData->Motor_Data.Right_Wheel_Motor_VEL * 1000) % 1000);
}

void Control_Data_Print(Classic_Data_t *pClassicData) {
    // 计算每个控制量的绝对值
    float fw_l_abs = 0.0f, fw_r_abs = 0.0f, tp_l_abs = 0.0f, tp_r_abs = 0.0f;
    float t1_l_abs = 0.0f, t2_l_abs = 0.0f, t1_r_abs = 0.0f, t2_r_abs = 0.0f;

    arm_abs_f32(&pClassicData->Contronller_Data.Fw_L, &fw_l_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.Fw_R, &fw_r_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.Tp_L, &tp_l_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.Tp_R, &tp_r_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.T1_L, &t1_l_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.T2_L, &t2_l_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.T1_R, &t1_r_abs, 1);
    arm_abs_f32(&pClassicData->Contronller_Data.T2_R, &t2_r_abs, 1);

    // 计算每个控制量的整数部分
    int fw_l_int = (int) (pClassicData->Contronller_Data.Fw_L * 1000) / 1000;
    int fw_r_int = (int) (pClassicData->Contronller_Data.Fw_R * 1000) / 1000;
    int tp_l_int = (int) (pClassicData->Contronller_Data.Tp_L * 1000) / 1000;
    int tp_r_int = (int) (pClassicData->Contronller_Data.Tp_R * 1000) / 1000;
    int t1_l_int = (int) (pClassicData->Contronller_Data.T1_L * 1000) / 1000;
    int t2_l_int = (int) (pClassicData->Contronller_Data.T2_L * 1000) / 1000;
    int t1_r_int = (int) (pClassicData->Contronller_Data.T1_R * 1000) / 1000;
    int t2_r_int = (int) (pClassicData->Contronller_Data.T2_R * 1000) / 1000;

    // 计算每个控制量的小数部分
    int fw_l_dec = (int) (fw_l_abs * 1000) % 1000;
    int fw_r_dec = (int) (fw_r_abs * 1000) % 1000;
    int tp_l_dec = (int) (tp_l_abs * 1000) % 1000;
    int tp_r_dec = (int) (tp_r_abs * 1000) % 1000;
    int t1_l_dec = (int) (t1_l_abs * 1000) % 1000;
    int t2_l_dec = (int) (t2_l_abs * 1000) % 1000;
    int t1_r_dec = (int) (t1_r_abs * 1000) % 1000;
    int t2_r_dec = (int) (t2_r_abs * 1000) % 1000;

    // 处理符号：当整数部分为0但原始值为负时，手动输出负号
    char fw_l_sign = (fw_l_int == 0 && pClassicData->Contronller_Data.Fw_L < 0.0f) ? '-' : ' ';
    char fw_r_sign = (fw_r_int == 0 && pClassicData->Contronller_Data.Fw_R < 0.0f) ? '-' : ' ';
    char tp_l_sign = (tp_l_int == 0 && pClassicData->Contronller_Data.Tp_L < 0.0f) ? '-' : ' ';
    char tp_r_sign = (tp_r_int == 0 && pClassicData->Contronller_Data.Tp_R < 0.0f) ? '-' : ' ';
    char t1_l_sign = (t1_l_int == 0 && pClassicData->Contronller_Data.T1_L < 0.0f) ? '-' : ' ';
    char t2_l_sign = (t2_l_int == 0 && pClassicData->Contronller_Data.T2_L < 0.0f) ? '-' : ' ';
    char t1_r_sign = (t1_r_int == 0 && pClassicData->Contronller_Data.T1_R < 0.0f) ? '-' : ' ';
    char t2_r_sign = (t2_r_int == 0 && pClassicData->Contronller_Data.T2_R < 0.0f) ? '-' : ' ';

    // 打印输出，按照Fw_L,Fw_R,Tp_L,Tp_R,T1_L,T2_L,T1_R,T2_R顺序
    uart_print("%c%d.%2d,%c%d.%2d,%c%d.%2d,%c%d.%2d,%c%d.%2d,%c%d.%2d,%c%d.%2d,%c%d.%2d\r\n",
               fw_l_sign, fw_l_int, fw_l_dec,
               fw_r_sign, fw_r_int, fw_r_dec,
               tp_l_sign, tp_l_int, tp_l_dec,
               tp_r_sign, tp_r_int, tp_r_dec,
               t1_l_sign, t1_l_int, t1_l_dec,
               t2_l_sign, t2_l_int, t2_l_dec,
               t1_r_sign, t1_r_int, t1_r_dec,
               t2_r_sign, t2_r_int, t2_r_dec);
}

void IMU_Data_Print(Classic_Data_t *pClassicData) {
    uart_print("roll:%d.%3d pitch:%d.%3d yaw:%d.%3d delta_roll:%d.%3d "
               "delta_pitch:%d.%3d delta_yaw:%d.%3d\r\n ",
               (int) (pClassicData->IMU_Data.roll * 1000) / 1000,
               (int) (pClassicData->IMU_Data.roll * 1000) % 1000,
               (int) (pClassicData->IMU_Data.pitch * 1000) / 1000,
               (int) (pClassicData->IMU_Data.pitch * 1000) % 1000,
               (int) (pClassicData->IMU_Data.yaw * 1000) / 1000,
               (int) (pClassicData->IMU_Data.yaw * 1000) % 1000,
               (int) (pClassicData->IMU_Data.delta_roll * 1000) / 1000,
               (int) (pClassicData->IMU_Data.delta_roll * 1000) % 1000,
               (int) (pClassicData->IMU_Data.delta_pitch * 1000) / 1000,
               (int) (pClassicData->IMU_Data.delta_pitch * 1000) % 1000,
               (int) (pClassicData->IMU_Data.delta_yaw * 1000) / 1000,
               (int) (pClassicData->IMU_Data.delta_yaw * 1000) % 1000);
}

void Classic_Data_Update(Classic_Data_t *pClassicData) {
    pClassicData->IMU_Data.roll = angle_to_rad(INS.Roll);
    pClassicData->IMU_Data.pitch = angle_to_rad(INS.Pitch);
    pClassicData->IMU_Data.yaw = angle_to_rad(INS.Yaw);
    pClassicData->Motor_Data.L1_Motor_POS =
            (DM_8009P1.data.position_rad / 3.50f * 3.14f) + 3.14f;
    pClassicData->Motor_Data.L2_Motor_POS =
            (DM_8009P2.data.position_rad / 3.50f * 3.14f) + 6.28f;
    pClassicData->Motor_Data.R1_Motor_POS =
            -(DM_8009P4.data.position_rad / 3.50f * 3.14f) - 3.14f;
    pClassicData->Motor_Data.R2_Motor_POS =
            -(DM_8009P3.data.position_rad / 3.50f * 3.14f);
    pClassicData->Motor_Data.L1_Motor_VEL = DM_8009P1.data.velocity_rad_s;
    pClassicData->Motor_Data.L2_Motor_VEL = DM_8009P2.data.velocity_rad_s;
    pClassicData->Motor_Data.R1_Motor_VEL = DM_8009P4.data.velocity_rad_s;
    pClassicData->Motor_Data.R2_Motor_VEL = DM_8009P3.data.velocity_rad_s;
    pClassicData->Motor_Data.Left_Wheel_Motor_VEL =
            -(DM_3519L.data.velocity_rad_s / 16.8f * 19.2f + 0.048f);
    pClassicData->Motor_Data.Right_Wheel_Motor_VEL =
            (DM_3519R.data.velocity_rad_s / 16.8f * 19.2f + 0.048f);

    // 计算时间差（微秒转秒），添加除零保护
    float time_diff = (pClassicData->Data_Time.imu_time -
                       pClassicData->Data_Time.last_imu_time) *
                      1e-6f;
    if (time_diff > 1e-6f) {
        // ====== 角速度计算（弧度制，带 ±π 跳变处理） ======
        float roll_change =
                pClassicData->IMU_Data.roll - pClassicData->IMU_Data.last_roll;
        if (roll_change > 3.14159f)
            roll_change -= 6.28318f;
        else if (roll_change < -3.14159f)
            roll_change += 6.28318f;
        pClassicData->IMU_Data.delta_roll =
                a * roll_change / time_diff +
                (1 - a) * pClassicData->IMU_Data.delta_roll;

        float pitch_change =
                pClassicData->IMU_Data.pitch - pClassicData->IMU_Data.last_pitch;
        if (pitch_change > 3.14159f)
            pitch_change -= 6.28318f;
        else if (pitch_change < -3.14159f)
            pitch_change += 6.28318f;
        pClassicData->IMU_Data.delta_pitch =
                a * pitch_change / time_diff +
                (1 - a) * pClassicData->IMU_Data.delta_pitch;

        float yaw_change =
                pClassicData->IMU_Data.yaw - pClassicData->IMU_Data.last_yaw;
        if (yaw_change > 3.14159f)
            yaw_change -= 6.28318f;
        else if (yaw_change < -3.14159f)
            yaw_change += 6.28318f;
        pClassicData->IMU_Data.delta_yaw =
                a * yaw_change / time_diff + (1 - a) * pClassicData->IMU_Data.delta_yaw;
    } else {
        pClassicData->IMU_Data.delta_roll = 0.0f;
        pClassicData->IMU_Data.delta_pitch = 0.0f;
        pClassicData->IMU_Data.delta_yaw = 0.0f;
    }
    pClassicData->States_Data.d_X =
            (pClassicData->Motor_Data.Left_Wheel_Motor_VEL * -0.08f);
    pClassicData->States_Data.X_pos =
            pClassicData->States_Data.X_pos + ((pClassicData->States_Data.d_X - d_X_ref) / 1000.f);
    if (pClassicData->States_Data.X_pos > -5.0f)
      pClassicData->States_Data.X_pos = -5.0f;
    if (pClassicData->States_Data.X_pos < -15.0f)
      pClassicData->States_Data.X_pos = -15.0f;

    pClassicData->States_Data.d_yaw = pClassicData->IMU_Data.delta_yaw + 0.0023f;
    pClassicData->States_Data.yaw +=
            (pClassicData->States_Data.d_yaw - d_yaw_ref) / 1000.f;
    pClassicData->States_Data.theta_L =
            +1.5708f - pClassicData->Leg_Data.phi_0_L - pClassicData->IMU_Data.pitch;
    pClassicData->States_Data.d_theta_L = -pClassicData->Leg_Data.delta_phi_0_L -
                                          pClassicData->IMU_Data.delta_pitch;
    pClassicData->States_Data.theta_R =
            +1.5708f - pClassicData->Leg_Data.phi_0_R - pClassicData->IMU_Data.pitch;
    pClassicData->States_Data.d_theta_R = -pClassicData->Leg_Data.delta_phi_0_R -
                                          pClassicData->IMU_Data.delta_pitch;
    pClassicData->States_Data.theta = pClassicData->IMU_Data.pitch;
    pClassicData->States_Data.d_theta = pClassicData->IMU_Data.delta_pitch;
}

void State_Data_Print(Classic_Data_t *pClassicData) {
    // 计算缩放后的绝对值
    float x_pos_abs = 0.0f;
    float d_x_abs = 0.0f;
    float yaw_abs = 0.0f;
    float d_yaw_abs = 0.0f;
    float theta_l_abs = 0.0f;
    float d_theta_l_abs = 0.0f;
    float theta_r_abs = 0.0f;
    float d_theta_r_abs = 0.0f;
    float theta_abs = 0.0f;
    float d_theta_abs = 0.0f;
    float dx_err = pClassicData->States_Data.d_X - pClassicData->Target_Data.Target_X_vel;
    // 使用ARMmath库的绝对值函数
    arm_abs_f32(&pClassicData->States_Data.X_pos, &x_pos_abs, 1);
    arm_abs_f32(&dx_err, &d_x_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.yaw_err, &yaw_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_yaw_err, &d_yaw_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.theta_L, &theta_l_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_theta_L, &d_theta_l_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.theta_R, &theta_r_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_theta_R, &d_theta_r_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.theta, &theta_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_theta, &d_theta_abs, 1);

    // 计算每个状态的整数部分
    int x_pos_int = (int) (pClassicData->States_Data.X_pos * 1000) / 1000;
    int d_x_int = (int) ((pClassicData->States_Data.d_X - pClassicData->Target_Data.Target_X_vel) * 1000) / 1000;
    int yaw_int = (int) (pClassicData->States_Data.yaw_err * 1000) / 1000;
    int d_yaw_int = (int) (pClassicData->States_Data.d_yaw_err * 1000) / 1000;
    int theta_l_int = (int) (pClassicData->States_Data.theta_L * 1000) / 1000;
    int d_theta_l_int = (int) (pClassicData->States_Data.d_theta_L * 1000) / 1000;
    int theta_r_int = (int) (pClassicData->States_Data.theta_R * 1000) / 1000;
    int d_theta_r_int = (int) (pClassicData->States_Data.d_theta_R * 1000) / 1000;
    int theta_int = (int) (pClassicData->States_Data.theta * 1000) / 1000;
    int d_theta_int = (int) (pClassicData->States_Data.d_theta * 1000) / 1000;

    // 计算每个状态的小数部分
    int x_pos_dec = (int) (x_pos_abs * 1000) % 1000;
    int d_x_dec = (int) (d_x_abs * 1000) % 1000;
    int yaw_dec = (int) (yaw_abs * 1000) % 1000;
    int d_yaw_dec = (int) (d_yaw_abs * 1000) % 1000;
    int theta_l_dec = (int) (theta_l_abs * 1000) % 1000;
    int d_theta_l_dec = (int) (d_theta_l_abs * 1000) % 1000;
    int theta_r_dec = (int) (theta_r_abs * 1000) % 1000;
    int d_theta_r_dec = (int) (d_theta_r_abs * 1000) % 1000;
    int theta_dec = (int) (theta_abs * 1000) % 1000;
    int d_theta_dec = (int) (d_theta_abs * 1000) % 1000;

    // 处理符号：当整数部分为0但原始值为负时，手动输出负号
    char x_pos_sign =
            (x_pos_int == 0 && pClassicData->States_Data.X_pos < 0.0f) ? '-' : ' ';
    char d_x_sign =
            (d_x_int == 0 && dx_err < 0.0f) ? '-' : ' ';
    char yaw_sign =
            (yaw_int == 0 && pClassicData->States_Data.yaw_err < 0.0f) ? '-' : ' ';
    char d_yaw_sign =
            (d_yaw_int == 0 && pClassicData->States_Data.d_yaw_err < 0.0f) ? '-' : ' ';
    char theta_l_sign =
            (theta_l_int == 0 && pClassicData->States_Data.theta_L < 0.0f)
                ? '-'
                : ' ';
    char d_theta_l_sign =
            (d_theta_l_int == 0 && pClassicData->States_Data.d_theta_L < 0.0f)
                ? '-'
                : ' ';
    char theta_r_sign =
            (theta_r_int == 0 && pClassicData->States_Data.theta_R < 0.0f)
                ? '-'
                : ' ';
    char d_theta_r_sign =
            (d_theta_r_int == 0 && pClassicData->States_Data.d_theta_R < 0.0f)
                ? '-'
                : ' ';
    char theta_sign =
            (theta_int == 0 && pClassicData->States_Data.theta < 0.0f) ? '-' : ' ';
    char d_theta_sign =
            (d_theta_int == 0 && pClassicData->States_Data.d_theta < 0.0f)
                ? '-'
                : ' ';

    // 打印输出，手动处理负号
    uart_print("%c%d.%d,%c%d.%d,%c%d.%d,%c%d.%d,%c%d.%d,%c%d.%d,%c%d.%d,%c%d.%d,%"
               "c%d.%d,%c%d.%d\r\n",
               x_pos_sign, x_pos_int, x_pos_dec, d_x_sign, d_x_int, d_x_dec,
               yaw_sign, yaw_int, yaw_dec, d_yaw_sign, d_yaw_int, d_yaw_dec,
               theta_l_sign, theta_l_int, theta_l_dec, d_theta_l_sign,
               d_theta_l_int, d_theta_l_dec, theta_r_sign, theta_r_int,
               theta_r_dec, d_theta_r_sign, d_theta_r_int, d_theta_r_dec,
               theta_sign, theta_int, theta_dec, d_theta_sign, d_theta_int,
               d_theta_dec);
}

void LQR_K_Calculate150(Classic_Data_t *pClassicData) {
    static arm_matrix_instance_f32 K_Matrix = {4, 10, (float *) K_Fixed_Leg150};
    static bool initialized = false;

    if (!initialized) {
        arm_mat_init_f32(&K_Matrix, 4, 10, (float *) K_Fixed_Leg150);
        initialized = true;
    }

    // 状态向量：使用偏差而不是绝对值
    float states[10] = {
        pClassicData->States_Data.X_pos, // 0: 位置偏差
        pClassicData->States_Data.d_x_err, // 1: 速度偏差
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.yaw_err,
        pClassicData->States_Data.d_yaw_err,
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.theta_L - theta_L_ref, // 2: 左腿角度偏差
        pClassicData->States_Data.d_theta_L - d_theta_L_ref, // 3: 左腿角速度偏差
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.theta_R - theta_R_ref, // 4:右腿角速度偏差
        pClassicData->States_Data.d_theta_R - d_theta_R_ref, // 5: 右腿角速度偏差
        // 0.0f,
        // 0.0f,
        -(pClassicData->States_Data.theta - theta_ref), // 6: 整体角度偏差
        -(pClassicData->States_Data.d_theta - d_theta_ref) // 7: 整体角速度偏差
        // 0.0f,
        // 0.0f,
    };

    float U_temp[4] = {0};
    arm_matrix_instance_f32 U = {4, 1, U_temp};

    arm_matrix_instance_f32 S = {10, 1, states};

    arm_mat_mult_f32(&K_Matrix, &S, &U);
    pClassicData->Contronller_Data.Fw_L = U_temp[0];
    pClassicData->Contronller_Data.Fw_R = U_temp[1];
    pClassicData->Contronller_Data.Tp_L = U_temp[2];
    pClassicData->Contronller_Data.Tp_R = U_temp[3];
}

void LQR_K_Calculate250(Classic_Data_t *pClassicData) {
    static arm_matrix_instance_f32 K_Matrix = {4, 10, (float *) K_Fixed_Leg250};
    static bool initialized = false;

    if (!initialized) {
        arm_mat_init_f32(&K_Matrix, 4, 10, (float *) K_Fixed_Leg250);
        initialized = true;
    }

    // 状态向量：使用偏差而不是绝对值
    float states[10] = {
        pClassicData->States_Data.X_pos, // 0: 位置偏差
        pClassicData->States_Data.d_x_err, // 1: 速度偏差
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.yaw_err,
        pClassicData->States_Data.d_yaw_err,
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.theta_L - theta_L_ref, // 2: 左腿角度偏差
        pClassicData->States_Data.d_theta_L - d_theta_L_ref, // 3: 左腿角速度偏差
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.theta_R - theta_R_ref, // 4:右腿角速度偏差
        pClassicData->States_Data.d_theta_R - d_theta_R_ref, // 5: 右腿角速度偏差
        // 0.0f,
        // 0.0f,
        -(pClassicData->States_Data.theta - theta_ref), // 6: 整体角度偏差
        -(pClassicData->States_Data.d_theta - d_theta_ref) // 7: 整体角速度偏差
        // 0.0f,
        // 0.0f,
    };

    float U_temp[4] = {0};
    arm_matrix_instance_f32 U = {4, 1, U_temp};

    arm_matrix_instance_f32 S = {10, 1, states};

    arm_mat_mult_f32(&K_Matrix, &S, &U);
    pClassicData->Contronller_Data.Fw_L = U_temp[0];
    pClassicData->Contronller_Data.Fw_R = U_temp[1];
    pClassicData->Contronller_Data.Tp_L = U_temp[2];
    pClassicData->Contronller_Data.Tp_R = U_temp[3];
}
void LQR_K_Calculate350(Classic_Data_t *pClassicData) {
    static arm_matrix_instance_f32 K_Matrix = {4, 10, (float *) K_Fixed_Leg350};
    static bool initialized = false;

    if (!initialized) {
        arm_mat_init_f32(&K_Matrix, 4, 10, (float *) K_Fixed_Leg350);
        initialized = true;
    }

    // 状态向量：使用偏差而不是绝对值
    float states[10] = {
        pClassicData->States_Data.X_pos, // 0: 位置偏差
        pClassicData->States_Data.d_x_err, // 1: 速度偏差
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.yaw_err,
        pClassicData->States_Data.d_yaw_err,
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.theta_L - theta_L_ref, // 2: 左腿角度偏差
        pClassicData->States_Data.d_theta_L - d_theta_L_ref, // 3: 左腿角速度偏差
        // 0.0f,
        // 0.0f,
        pClassicData->States_Data.theta_R - theta_R_ref, // 4:右腿角速度偏差
        pClassicData->States_Data.d_theta_R - d_theta_R_ref, // 5: 右腿角速度偏差
        // 0.0f,
        // 0.0f,
        -(pClassicData->States_Data.theta - theta_ref), // 6: 整体角度偏差
        -(pClassicData->States_Data.d_theta - d_theta_ref) // 7: 整体角速度偏差
        // 0.0f,
        // 0.0f,
    };

    float U_temp[4] = {0};
    arm_matrix_instance_f32 U = {4, 1, U_temp};

    arm_matrix_instance_f32 S = {10, 1, states};

    arm_mat_mult_f32(&K_Matrix, &S, &U);
    pClassicData->Contronller_Data.Fw_L = U_temp[0];
    pClassicData->Contronller_Data.Fw_R = U_temp[1];
    pClassicData->Contronller_Data.Tp_L = U_temp[2];
    pClassicData->Contronller_Data.Tp_R = U_temp[3];
}


// 输出限幅（VMC 之后）：T1/T2；Fw 不经过 VMC，也在此限幅
void Controler_Limit(Classic_Data_t *pClassicData) {
  // Fw 限幅：±6.0 N（轮子力，不经过 VMC）
  if (pClassicData->Contronller_Data.Fw_L > 6.0f)
    pClassicData->Contronller_Data.Fw_L = 6.0f;
  if (pClassicData->Contronller_Data.Fw_L < -6.0f)
    pClassicData->Contronller_Data.Fw_L = -6.0f;
  if (pClassicData->Contronller_Data.Fw_R > 6.0f)
    pClassicData->Contronller_Data.Fw_R = 6.0f;
  if (pClassicData->Contronller_Data.Fw_R < -6.0f)
    pClassicData->Contronller_Data.Fw_R = -6.0f;
  // T1/T2 限幅：±20.5 N·m
  if (pClassicData->Contronller_Data.T1_L > 20.5f)
    pClassicData->Contronller_Data.T1_L = 20.5f;
  if (pClassicData->Contronller_Data.T1_L < -20.5f)
    pClassicData->Contronller_Data.T1_L = -20.5f;
  if (pClassicData->Contronller_Data.T1_R > 20.5f)
    pClassicData->Contronller_Data.T1_R = 20.5f;
  if (pClassicData->Contronller_Data.T1_R < -20.5f)
    pClassicData->Contronller_Data.T1_R = -20.5f;
  if (pClassicData->Contronller_Data.T2_L > 20.5f)
    pClassicData->Contronller_Data.T2_L = 20.5f;
  if (pClassicData->Contronller_Data.T2_L < -20.5f)
    pClassicData->Contronller_Data.T2_L = -20.5f;
  if (pClassicData->Contronller_Data.T2_R > 20.5f)
    pClassicData->Contronller_Data.T2_R = 20.5f;
  if (pClassicData->Contronller_Data.T2_R < -20.5f)
    pClassicData->Contronller_Data.T2_R = -20.5f;
}