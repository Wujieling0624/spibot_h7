#ifndef _GQ_MOTOR_H
#define _GQ_MOTOR_H
#include "bsp_can.h"
#include "motor_def.h"

#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"

#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float current;
    int32_t total_round;
    float total_angle;
}GQ_Motor_Measure_s;

typedef enum
{
    gq_current_mode = 0,
    gq_angle_mode = 1,
    gq_stop_mode = 3,
}GQMode_type_e;


typedef struct 
{
    GQ_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    Motor_CloseLoop_Type_e motor_close_loop_type;
    Closeloop_Type_e close_loop_type;
    Closeloop_Type_e outer_loop_type;


    Motor_Type_e motor_type;

}GQMotorInstance;

GQMotorInstance *GQMotorInit(Motor_Init_Config_s *config);



void GQMotor_Zero_force(GQMotorInstance *motor);
void GQMotorEnable(GQMotorInstance *motor);
void GQMotor_AngleMode_SetPD(GQMotorInstance *motor, float Kp, float Kd);
void GQMotorControlInit();
void GQMotorTask();
void GQMotorStop(GQMotorInstance *motor);
void GQMotorGetMeasure(GQMotorInstance *motor);
void set_pos_vel_tqe_pd_int16(GQMotorInstance *motor,  int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

void GQMotor_Setref(GQMotorInstance *motor, float ref);
#endif