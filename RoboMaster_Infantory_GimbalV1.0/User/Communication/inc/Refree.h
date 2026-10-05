#ifndef USER_COMMUNICATION_REFREE_H
#define USER_COMMUNICATION_REFREE_H
#include <stdint.h>
#include "Ring_Buffer.h"
#include "cmsis_os2.h"

/* V1.7.0 图传链路中本步兵接收的标准 UART 命令。 */
#define REFEREE_VTM_CMD_CUSTOM_CONTROLLER 0x0302U /* 自定义控制器通过图传发送给机器人的命令 ID。 */
#define REFEREE_VTM_CMD_KEYBOARD_MOUSE    0x0304U /* 官方键鼠遥控命令 ID。 */
#define REFEREE_VTM_KEYBOARD_MOUSE_LEN    12U     /* 0x0304 键鼠载荷长度。 */
#define REFEREE_VTM_CUSTOM_DATA_MAX_LEN   30U     /* 自定义控制器载荷长度。 */
/* 官方图传链路：0x0304 键鼠以固定 30 Hz 发送，约每 33.3 ms 一帧。
 * 此处只描述通信协议节拍；掉线判定属于 ModeTask，见
 * MODE_KEYBOARD_MOUSE_TIMEOUT_MS。 */
#define REFEREE_VTM_KEYBOARD_MOUSE_HZ        (30U)
#define REFEREE_VTM_KEYBOARD_MOUSE_PERIOD_MS (1000U / REFEREE_VTM_KEYBOARD_MOUSE_HZ)
/* 0x0304 keyboard_value 位定义（RoboMaster 裁判协议，按住时对应位为 1）。
 * 每个按键在 keyboard_value 中有独立 bit，接收后保留整个 16-bit 位图；
 * 绝不把不同按键混成同一个无来源的标志。例：
 *   (keyboard_value & REFEREE_KEY_F) != 0  表示 F 当前按住。 */
#define REFEREE_KEY_W       (1U << 0)
#define REFEREE_KEY_S       (1U << 1)
#define REFEREE_KEY_A       (1U << 2)
#define REFEREE_KEY_D       (1U << 3)
#define REFEREE_KEY_SHIFT   (1U << 4)
#define REFEREE_KEY_CTRL    (1U << 5)
#define REFEREE_KEY_Q       (1U << 6)
#define REFEREE_KEY_E       (1U << 7)
#define REFEREE_KEY_R       (1U << 8)
#define REFEREE_KEY_F       (1U << 9)
#define REFEREE_KEY_G       (1U << 10)
#define REFEREE_KEY_Z       (1U << 11)
#define REFEREE_KEY_X       (1U << 12)
#define REFEREE_KEY_C       (1U << 13)
#define REFEREE_KEY_V       (1U << 14)
#define REFEREE_KEY_B       (1U << 15)

/*
 * 最新一帧自定义控制数据。
 * 图传控制属于实时状态量，因此新帧覆盖旧帧；业务层通过 update_tick
 * 判断图传是否失联，而不应积压过期控制命令。
 * 本结构仅保存 0x0302 的 30 字节原始数据，业务字节含义由控制层定义。
 */
typedef struct {
  uint8_t data[REFEREE_VTM_CUSTOM_DATA_MAX_LEN]; /* 原始自定义载荷，未解释其业务字节。 */
  uint8_t length;                                /* 本帧有效数据长度。 */
  uint8_t sequence;                              /* 裁判帧头 seq，用于抓包和丢帧诊断。 */
  uint32_t update_tick;                          /* 最近一次有效帧到达的 HAL ms 时基。 */
  uint32_t frame_count;                          /* 此类别已接收的有效帧总数。 */
} Referee_VtmCustomRx_t;

/* UART7 图传接收链路的诊断统计，供调试器或上位机读取。 */
typedef struct {
  uint32_t dma_bytes;              /* UART7 DMA 成功写入软件环形缓冲的字节数。 */
  uint32_t ring_overflow_count;    /* InputTask 未及时消费导致的软件环形缓冲溢出次数。 */
  uint32_t frame_count;            /* CRC8/长度/CRC16 均通过的完整裁判帧数。 */
  uint32_t crc8_error_count;       /* 帧头 CRC8 校验失败次数。 */
  uint32_t crc16_error_count;      /* 整帧 CRC16 校验失败次数。 */
  uint32_t length_error_count;     /* data_length 超过协议最大长度或命令长度不匹配次数。 */
  uint32_t unsupported_cmd_count;  /* 合法但当前步兵不处理的 cmd_id 数量。 */
  uint32_t event_queue_overflow_count; /* 保序事件队列满，最新事件未入队次数。 */
} Referee_VtmStats_t;

/* Exactly 40 bytes: a FIFO-preserved one-shot custom-controller command.
 * Continuous keyboard/mouse state remains a latest-value snapshot. */
typedef struct {
  uint8_t type;
  uint8_t sequence;
  uint8_t length;
  uint8_t reserved;
  uint32_t timestamp_ms;
  uint8_t data[REFEREE_VTM_CUSTOM_DATA_MAX_LEN];
  uint8_t padding[2];
} Referee_Event_t;
_Static_assert(sizeof(Referee_Event_t) == 40U,
               "Referee_Data_Queue item size must remain 40 bytes");

/*
 * 轮腿步兵当前需要的图传输入快照，包含三层互不覆盖的数据：
 * 1. keyboard_mouse：官方 0x0304 原样拆出的每个字段；
 * 2. custom_controller：0x0302 原始自定义载荷，不参与键鼠映射。
 * 通信模块不解释摇杆/鼠标/按键的业务含义，ModeTask 是唯一解释者。
 */
typedef struct {
  int16_t mouse_x;            /* 官方每帧鼠标 X 增量，不是累计位置。 */
  int16_t mouse_y;            /* 官方每帧鼠标 Y 增量，不是累计位置。 */
  int16_t mouse_z;            /* 官方每帧滚轮增量，不是累计位置。 */
  uint8_t left_button_down;   /* 左键状态：0 松开，1 按下。 */
  uint8_t right_button_down;  /* 右键状态：0 松开，1 按下。 */
  uint16_t keyboard_value;    /* 所有 16 个键独立保存的位图，见 REFEREE_KEY_*。 */
  uint16_t reserved;          /* V1.7 协议保留字节。 */
  /* 位图的逐键展开。保留位图是为了追溯协议，保留这些字段是为了业务层
   * 无须再次猜测 bit 号；它们均为当前帧“是否按住”，不是按下沿事件。 */
  uint8_t key_w, key_s, key_a, key_d;
  uint8_t key_shift, key_ctrl, key_q, key_e;
  uint8_t key_r, key_f, key_g, key_z;
  uint8_t key_x, key_c, key_v, key_b;
  uint32_t update_tick;       /* 最近有效帧到达的 HAL ms 时基。 */
  uint32_t frame_count;       /* 已接收有效键鼠帧总数。 */
} Referee_VtmKeyboardMouse_t;

typedef struct {
  Referee_VtmKeyboardMouse_t keyboard_mouse; /* 0x0304，官方键鼠 -> 机器人 */
  Referee_VtmCustomRx_t custom_controller; /* 0x0302，自定义控制器 -> 机器人 */
  Referee_VtmStats_t stats;
} Referee_VtmInput_t;
_Static_assert(sizeof(Referee_VtmInput_t) == 108U,
               "Mode_Referee_Queue element size must remain 108 bytes");

/** @brief 初始化图传软件缓存并启动 UART7 Receive-to-Idle DMA。
 * 不解析业务含义，不发布任何控制指令。 */
void Refree_Init(void);

/**
 * @brief 消费 UART7 软件环形缓冲，完成裁判帧同步、校验与命令分类。
 * @note 由 InputTask 每 1 ms 调用；首次调用会执行 Refree_Init()。
 * 对 0x0304：只完整保存鼠标/左右键/全部键盘位图，不生成业务指令。
 * 对 0x0302：只保留原始载荷，绝不与 0x0304 字段混用。
 */
void Refree_Process(void);

/**
 * @brief UART7 DMA 空闲/满回调入口。
 * @param data DMA 原始缓冲的有效数据首地址。
 * @param size 本次 DMA 接收的有效字节数。
 * @note 只能从 ISR 调用；本函数只写软件环形缓冲，不做协议解析、按键映射或仲裁。
 */
void Refree_UART7_DmaRxEvent(const uint8_t *data, uint16_t size);

/** 按先进先出顺序获取下一条自定义控制器事件。 */
osStatus_t Refree_GetEvent(Referee_Event_t *out, uint32_t timeout_ms);
extern osMessageQueueId_t Mode_Referee_QueueHandle;
#endif
