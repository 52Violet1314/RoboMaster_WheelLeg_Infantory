#ifndef __APP_INSTASK_HPP__
#define __APP_INSTASK_HPP_

/* 置 1：每次 MCU 启动时向 HI14 写入 X右/Y前/Z上（URFR 24）并保存。
 * IMU 已确认配置后改为 0，避免每次启动重复写 Flash。 */
#define HI14_APPLY_URFR24_ON_BOOT 1U

void app_INSTask(void *pvParameters);
void HI14_ApplyURFR24Configuration(void);

#endif // __APP_INSTASK_HPP__
