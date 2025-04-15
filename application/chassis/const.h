#ifndef CONST_H
#define CONST_H

#include "math.h"
#include <stdbool.h>
#include "main.h"

#define toAngle (pi/180.0f)
#define pi 3.141592653589793f
#define T 120
#define r 0.15f
#define T1 T / 3.0
#define T2 T / 2.0
#define w1 (2 * pi) / (1.0 * T1)
#define w2 (2 * pi) / (1.0 * T2)
#define l1 0.05
#define l2 0.23
#define l3 0.33

float rad[12] = {0}, x_pos[4] = {0}, y_pos[4] = {0}, z_pos[4] = {0};

bool supportPhase[4] = {false};
static bool legMotor[4] = {false}; // 0:fr 1:br 2:bl 3:fl 
static bool all_motors_ready = false;

typedef struct {
    float x, y, z;
} Vector3;
Vector3 endV[4]; // 存储四个末端法向量

typedef struct {
    float x, y, z;
} Offset;
Offset offset = {0.2f, 0.16f, 0.25f};

// Motor instances
typedef struct {
    GQMotorInstance *hip;
    GQMotorInstance *thigh;
    GQMotorInstance *shank;
} LegMotors;
LegMotors Leg[4]; //  0:fr 1:br 2:bl 3:fl 

// 角度赋值的前置符号
static const int joint_sign[4][3] = 
{
    { 1, -1, -1 }, // fr
    { 1,  1,  1 }, // br
    { -1, -1, -1 }, // bl
    { -1,  1,  1 }  //fl
};

int n = 0;
float theta1, theta2, theta3;

//  0:fr 1:br 2:bl 3:fl 
float xd[4] = {0.0};
float yd[4] = {0.0};
float zd[4] = {0.0};
float theta1d[4] = {0.0};
float theta2d[4] = {0.0};
float theta3d[4] = {0.0};

float t = 0.0, _t = 0.0;
bool spibot_init = false;
bool stand_ready = false;

 // 0:fr 1:br 2:bl 3:fl 
float init_hipAngle[4] = {0.0};
float init_thighAngle[4] = {0.0};
float init_shankAngle[4] = {0.0};
float hipAngle[4] = {0.0};
float thighAngle[4] = {0.0};
float shankAngle[4] = {0.0};

static void Pump_on() { HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin | POWER_24V_1_Pin, GPIO_PIN_SET); }
static void Pump_off() { HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin | POWER_24V_1_Pin, GPIO_PIN_RESET); }
static void FRSolenoidValve_on() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET); }
static void FRSolenoidValve_off() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET); }
static void FLSolenoidValve_on() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); }
static void FLSolenoidValve_off() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); }
static void BRSolenoidValve_on() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); }
static void BRSolenoidValve_off() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); }
static void BLSolenoidValve_on() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); }
static void BLSolenoidValve_off() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); }




#endif