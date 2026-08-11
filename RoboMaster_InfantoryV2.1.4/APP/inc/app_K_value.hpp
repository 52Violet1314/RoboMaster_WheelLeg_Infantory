#ifndef __APP_K_VALUE_HPP__
#define __APP_K_VALUE_HPP__
typedef float K_Fixed_Leg_t[4][10];

typedef struct
{
    K_Fixed_Leg_t fixed_leg;
} K_Unfixed_Leg_t;

extern K_Fixed_Leg_t K_Fixed_Leg350;
extern K_Fixed_Leg_t K_Fixed_Leg250;
extern K_Fixed_Leg_t K_Fixed_Leg150;


#endif