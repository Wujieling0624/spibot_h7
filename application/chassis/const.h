#ifndef CONST_H
#define CONST_H

#include "robot_def.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "GQmotor.h"
#include "arm_math.h"
#include "gqmotor.h"
#include "rm_referee.h"
#include "math.h"
#include <stdbool.h>
#include "main.h"

#define pi 3.141592653589793
#define T 120
#define r 0.15f
#define T1 T / 3.0
#define T2 T / 2.0
#define w1 (2 * pi) / (1.0 * T1)
#define w2 (2 * pi) / (1.0 * T2)
#define l1 0.05
#define l2 0.23
#define l3 0.33
#define toRad (pi/180.0)
#define toAngle (180.0/pi)

double rad[12] = {0};

static bool all_motors_ready = false;

typedef struct {
    double x, y, z;
} Vector3;

typedef struct {
    float x, y, z;
} Offset;
Offset offset = {0.2f, 0.16f, 0.25f};

// 0:fr 1:br 2:bl 3:fl 
typedef struct {
    GQMotorInstance *hip, *thigh, *shank;
    bool swingPhase;
    bool legMotorReady;
    double endEffector_posX, endEffector_posY, endEffector_posZ;
    Vector3 endV; // 四个末端法向量
} LegMotors;
LegMotors Leg[4]; 

// 角度赋值的前置符号
static const int joint_sign[4][3] = {
    { 1, -1, -1 }, // fr
    { 1,  1,  1 }, // br
    { -1, -1, -1 }, // bl
    { -1,  1,  1 }  //fl
};

int n = 0;

float _t = 0.0;
bool spibot_init = false;
bool stand_ready = false;

 // 0:fr 1:br 2:bl 3:fl 
float init_hipAngle[4] = {0.0};
float init_thighAngle[4] = {0.0};
float init_shankAngle[4] = {0.0};
float hipAngle[4] = {0.0};
float thighAngle[4] = {0.0};
float shankAngle[4] = {0.0};
double xd[4] = {0.0};
double yd[4] = {0.0};
double zd[4] = {0.0};
double theta1d[4] = {0.0};
double theta2d[4] = {0.0};
double theta3d[4] = {0.0};
double theta1[4] = {0.0};
double theta2[4] = {0.0};
double theta3[4] = {0.0};

void Pump_on() { HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin | POWER_24V_1_Pin, GPIO_PIN_SET); }
void Pump_off() { HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin | POWER_24V_1_Pin, GPIO_PIN_RESET); }
void FRSolenoidValve_on() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET); }
void FRSolenoidValve_off() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET); }
void FLSolenoidValve_on() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); }
void FLSolenoidValve_off() { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); }
void BRSolenoidValve_on() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); }
void BRSolenoidValve_off() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); }
void BLSolenoidValve_on() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); }
void BLSolenoidValve_off() { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); }

#endif