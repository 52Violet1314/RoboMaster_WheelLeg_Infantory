#ifndef USER_COMMUNICATION_HIPNUC_COMMAND_H
#define USER_COMMUNICATION_HIPNUC_COMMAND_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* ============================================================
 * HiPNUC IMU ASCII 配置命令库
 *
 * 本模块只通过 USART2 的 TX 向 IMU 发送以 \r\n 结尾的 ASCII 命令；
 * 不参与 HI91 二进制接收、CRC 校验和姿态解析。
 *
 * 使用限制：
 * 1. 只能在任务上下文调用，绝不能在中断或 1 kHz 控制路径调用；
 * 2. 配置期间必须让电机保持零输出；
 * 3. 命令回复会和 HI91 流同时到达 USART2 RX，现有 HI91 解析器会丢弃
 *    ASCII 文本并自动重新同步。因此本库只报告“字节是否发出”，不解析 OK；
 * 4. SERIALCONFIG 会立即切换 IMU 波特率，主机 USART2 必须紧接着切换。
 *
 * 当前工程推荐的固定配置（本机器人）：
 *   Hipnuc_Command_SetUrfr(134U);                 // 模块轴：X 左、Y 后、Z 上
 *   Hipnuc_Command_SetWorldCoordinate(HIPNUC_COORD_ENU); // 世界系：ENU
 *   Hipnuc_Command_SetAttitudeMode(HIPNUC_ATT_MODE_AHRS_9_AXIS); // 屏蔽壳、完成校准后用 9 轴
 *   Hipnuc_Command_SetHi91PeriodMs(1U);           // HI91 每 1 ms 一帧，即 1 kHz
 *   Hipnuc_Command_Save();
 *   Hipnuc_Command_Reboot();
 *
 * 上述配置只在首次装机、IMU 安装方向改变或需要切换工作模式时发送。
 * 配置已保存后，日常上电只启动 HI91 接收，不需要重复发送。
 * ============================================================ */

#define HIPNUC_COMMAND_TIMEOUT_MS (50U)
#define HIPNUC_HI91_MIN_PERIOD_MS (1U)
#define HIPNUC_HI91_MAX_PERIOD_MS (1000U)

/* 首次配置开关：默认必须为 0。
 * 仅在下列前提同时满足时，临时改为 1 并烧录一次：
 * 1. IMU 当前串口已是本工程 USART2 的 921600、8N1；
 * 2. 电机断使能且机器人静止；
 * 3. 已确认 IMU 物理安装为 X 左、Y 后、Z 上。
 * 本次启动会写入 IMU 非易失配置并重启它；完成后立刻改回 0，日常不得重复写入。
 * 具体调用位置在 Task_IMU_Task：USART2 DMA 接收启动前，避免 ASCII 回复和 HI91 混流。 */
#define HIPNUC_FIRST_USE_CONFIG_ON_BOOT (0U)

/* 发送 REBOOT 后 IMU 的最小等待时间；期间不能启动 HI91 DMA 接收或控制云台。 */
#define HIPNUC_REBOOT_WAIT_MS (500U)

typedef enum {
  HIPNUC_ATT_MODE_VRU_6_AXIS = 0U,
  /* 填 0：6 轴陀螺+加速度计。
   * 推荐本工程使用：室内、云台、电机和大电流线附近磁干扰大；Yaw 为相对上电零点。 */
  HIPNUC_ATT_MODE_AHRS_9_AXIS = 1U,
  /* 填 1：9 轴加入磁力计，Yaw 参考磁北。
   * 本工程当前推荐此值：IMU 有屏蔽壳；首次整机安装后必须完成地磁校准。
   * 屏蔽壳只能减小电机干扰，不能替代校准；若实测 Yaw 受电机开关明显跳变，改回 6 轴。 */
  HIPNUC_ATT_MODE_HUMANOID = 4U,          /* 填 4：厂家人形机器人专用 profile。 */
  HIPNUC_ATT_MODE_LOW_SPEED_GROUND = 5U,  /* 填 5：低速地面平台专用 profile。 */
  HIPNUC_ATT_MODE_LOW_DYNAMIC = 7U        /* 填 7：低动态/倾角仪专用 profile。 */
} Hipnuc_AttitudeMode_t;

typedef enum {
  HIPNUC_COORD_ENU = 0U,
  /* 填 0：东-北-天世界坐标系，说明书默认值。
   * 本工程必须使用 ENU：HI91 中 pitch 绕用户 X 轴，云台取 gyro_x 作为 Pitch 角速度。 */
  HIPNUC_COORD_NWU = 4U
  /* 填 4：北-西-天世界坐标系。
   * 此模式 roll/pitch 的绕轴定义与 ENU 不同；当前云台和视觉代码不能直接切换到此值。 */
} Hipnuc_WorldCoordinate_t;

/**
 * @brief 发送一条完整原始 ASCII 命令。
 * @param command 完整命令，必须包含结尾 "\r\n"，例如 "LOG VERSION\r\n"；最长 95 字节。
 * @return HAL_OK 仅表示 USART2 已接受待发送字节；不表示 IMU 已返回 OK 或配置成功。
 */
HAL_StatusTypeDef Hipnuc_Command_SendRaw(const char *command);
/**
 * @brief 开关当前 USART2 上的全部数据帧输出。
 * @param enabled 0=发送 "LOG DISABLE" 停止所有输出；1=发送 "LOG ENABLE" 恢复输出。
 * @note 首次改波特率或用串口工具配置前可先填 0，避免 HI91 二进制流干扰查看 ASCII 回复。
 */
HAL_StatusTypeDef Hipnuc_Command_SetLogEnabled(uint8_t enabled);
/**
 * @brief 配置 HI91 的定时输出周期。
 * @param period_ms 1~1000：周期单位 ms；填 1=1 kHz，10=100 Hz，20=50 Hz，100=10 Hz；填 0=关闭 HI91。
 * @note 本工程填 1，且 USART2 与 IMU 必须均为 921600、8N1。
 */
HAL_StatusTypeDef Hipnuc_Command_SetHi91PeriodMs(uint16_t period_ms);
/**
 * @brief 设置当前连接串口的波特率。
 * @param baud 只允许填 4800/9600/19200/38400/57600/115200/230400/460800/921600。
 * @note 本工程填 921600。命令发出后 IMU 立即切换，STM32 必须随后重新配置 USART2，
 *       否则下一条命令无法通信；不能在 1 kHz 控制过程中调用。
 */
HAL_StatusTypeDef Hipnuc_Command_SetBaud(uint32_t baud);
/**
 * @brief 设置模块安装方向 URFR 编码。
 * @param urfr_code 按说明书 24 种合法右手安装填写。常用值：
 *        24=模块 X右/Y前/Z上（默认水平）；134=模块 X左/Y后/Z上（本工程）；
 *        205=模块 X前/Y右/Z下；520=模块 X上/Y前/Z左。
 * @note 本工程必须填 134。填入的是“IMU 物理 X/Y/Z 轴在车体中的实际方向”，
 *       不是世界坐标方向。改动安装方式后必须重新检查 Pitch 与 gyro_x 正负号。
 */
HAL_StatusTypeDef Hipnuc_Command_SetUrfr(uint16_t urfr_code);
/**
 * @brief 设置 ENU/NWU 世界坐标系。
 * @param coordinate 填 HIPNUC_COORD_ENU(0) 或 HIPNUC_COORD_NWU(4)。
 * @note 本工程只能填 HIPNUC_COORD_ENU。该配置描述世界参考轴，不等于安装方向；
 *       保存并重启后生效，改动后必须同步修改欧拉角解释和云台轴映射。
 */
HAL_StatusTypeDef Hipnuc_Command_SetWorldCoordinate(Hipnuc_WorldCoordinate_t coordinate);
/**
 * @brief 设置姿态/航向工作模式。
 * @param mode 填 Hipnuc_AttitudeMode_t 枚举值；本工程有屏蔽壳时填 HIPNUC_ATT_MODE_AHRS_9_AXIS(1)。
 * @note 模式 1 的绝对航向依赖磁力计，安装/磁环境变化后应重新做地磁校准。
 *       该长期配置保存并重启后加载，不能在云台运行中切换。
 */
HAL_StatusTypeDef Hipnuc_Command_SetAttitudeMode(Hipnuc_AttitudeMode_t mode);
/** @brief 保存当前配置，或命令 IMU 重启。配置 URFR、世界系、模式、HI91 周期后应按此顺序调用。 */
HAL_StatusTypeDef Hipnuc_Command_Save(void);
HAL_StatusTypeDef Hipnuc_Command_Reboot(void);
/**
 * @brief 查询模块版本、当前通信输出配置或用户配置。
 * @note 当前工程只发送查询命令、不解析 ASCII 回包；需读取结果时使用 CHCenter/串口工具，
 *       或后续单独增加 ASCII 命令回复解析器。
 */
HAL_StatusTypeDef Hipnuc_Command_QueryVersion(void);
HAL_StatusTypeDef Hipnuc_Command_QueryComConfig(void);
HAL_StatusTypeDef Hipnuc_Command_QueryUserConfig(void);
/**
 * @brief 9 轴模式下启动地磁校准。
 * @param planar_2d 0=3D 校准（各轴旋转，推荐完整机器人）；1=2D 校准（基本水平，仅绕竖直轴）。
 * @note 仅 9 轴模式使用。必须在电机停止、磁环境干净、IMU 与机器人固定装配后执行；
 *       日常启动不得自动调用。校准期间与结束后用 Hipnuc_Command_QueryMagCalibration 查询。
 */
HAL_StatusTypeDef Hipnuc_Command_StartMagCalibration(uint8_t planar_2d);
HAL_StatusTypeDef Hipnuc_Command_QueryMagCalibration(void);

#endif
