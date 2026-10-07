#include "Fric.h"
#include "Can_Motor.h"
#include "Mode.h"

void Fric_Control_Stop(void)
{
#if (CAN_MOTOR_LEFT_PRESENT != 0U)
  (void)Can_Motor_LeftFrictionSpeed(0.0f);
#endif
#if (CAN_MOTOR_RIGHT_PRESENT != 0U)
  (void)Can_Motor_RightFrictionSpeed(0.0f);
#endif
}

void Fric_Control_UpdateLeft(uint8_t requested_on, uint8_t allowed)
{
#if (CAN_MOTOR_LEFT_PRESENT != 0U)
  const float velocity_rad_s = (requested_on != 0U && allowed != 0U) ?
      -MODE_FRIC_WHEEL_SPEED_RAD_S : 0.0f;
  (void)Can_Motor_LeftFrictionSpeed(velocity_rad_s);
#else
  (void)requested_on;
  (void)allowed;
#endif
}

void Fric_Control_UpdateRight(uint8_t requested_on, uint8_t allowed)
{
#if (CAN_MOTOR_RIGHT_PRESENT != 0U)
  const float velocity_rad_s = (requested_on != 0U && allowed != 0U) ?
      MODE_FRIC_WHEEL_SPEED_RAD_S : 0.0f;
  (void)Can_Motor_RightFrictionSpeed(velocity_rad_s);
#else
  (void)requested_on;
  (void)allowed;
#endif
}
