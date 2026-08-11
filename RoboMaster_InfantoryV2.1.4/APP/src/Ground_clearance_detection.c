#include "Ground_clearance_detection.h"
#include "CalculateTask.hpp"
#include "arm_math.h"

#define G 9.81f
#define Mass 1.0f

float Ground_Clearance_Detection(float F0,float Tp,float L0,float d_L0,float dd_L0,float dd_zm,float phi,float d_phi,float dd_phi)
{
    float Fn = F0 * arm_cos_f32(phi) + (Tp * arm_sin_f32(phi)) / L0 + Mass * (G + dd_zm - dd_L0*arm_cos_f32(phi) + 2*d_L0*d_phi*arm_sin_f32(phi));
}
