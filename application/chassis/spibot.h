#ifndef SPIBOT_H
#define SPIBOT_H

#include "robot_def.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "GQmotor.h"
#include "arm_math.h"
#include "gqmotor.h"
#include "rm_referee.h"
#include "forward_traj.h"

void InitAngleSet()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (Leg[i].hip->measure.current != 0 && Leg[i].thigh->measure.current != 0 && Leg[i].shank->measure.current != 0)
        {
            GQMotorEnable(Leg[i].hip);
            GQMotorEnable(Leg[i].thigh);
            GQMotorEnable(Leg[i].shank);
            init_hipAngle[i] = Leg[i].hip->measure.position;
            init_thighAngle[i] = Leg[i].thigh->measure.position;
            init_shankAngle[i] = Leg[i].shank->measure.position;
            GQMotor_Setref(Leg[i].hip, init_hipAngle[i]);
            GQMotor_Setref(Leg[i].thigh, init_thighAngle[i]);
            GQMotor_Setref(Leg[i].shank, init_shankAngle[i]);
            legMotor[i] = true;
        }
    }
}

void PosToTheta(float xd, float yd, float zd)
{
    float L = sqrt(pow(xd, 2) + pow(yd, 2));
    theta1 = atan2(xd, yd);       // 弧度
    float ar = atan2(zd, L - l1); // 弧度
    float Lr = sqrt(pow(zd, 2) + pow((L - l1), 2));
    float a1 = acos((pow(Lr, 2) + pow(l2, 2) - pow(l3, 2)) / (2.0 * l2 * Lr)); // 弧度
    float a2 = acos((pow(Lr, 2) + pow(l3, 2) - pow(l2, 2)) / (2.0 * Lr * l3)); // 弧度
    theta2 = a1 - ar;                                                          // 弧度
    theta3 = -90 / 180.0 * pi + a1 + a2;                                       // 弧度
    theta1 = (theta1 / pi) * 180;
    theta2 = (theta2 / pi) * 180;
    theta3 = (theta3 / pi) * 180;
}

void motorReadyCheck(){
    InitAngleSet();
    if (legMotor[0] && legMotor[1] && legMotor[2] && legMotor[3])
    {
        all_motors_ready = true;
    }
}

void Joint2Theta(uint8_t leg_id)
{
    PosToTheta(xd[leg_id], yd[leg_id], zd[leg_id]);
    theta1d[leg_id] = joint_sign[leg_id][0] * theta1;
    theta2d[leg_id] = joint_sign[leg_id][1] * theta2;
    theta3d[leg_id] = joint_sign[leg_id][2] * theta3;
}

void GQ_motorSet()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        GQMotor_Setref(Leg[i].hip, theta1d[i]);
        GQMotor_Setref(Leg[i].thigh, theta2d[i]);
        GQMotor_Setref(Leg[i].shank, theta3d[i]);
    }
}

void SpibotStand()
{
    if (fabs(theta2d[0] - thighAngle[0]) >= 0.5)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            hipAngle[i] = init_hipAngle[i] + _t * (theta1d[i] - init_hipAngle[i]) / 10.0;
            thighAngle[i] = init_thighAngle[i] + _t * (theta2d[i] - init_thighAngle[i]) / 10.0;
            shankAngle[i] = init_shankAngle[i] + _t * (theta3d[i] - init_shankAngle[i]) / 10.0;
            GQMotor_Setref(Leg[i].hip, hipAngle[i]);
            GQMotor_Setref(Leg[i].thigh, thighAngle[i]);
            GQMotor_Setref(Leg[i].shank, shankAngle[i]);
        }
        _t += 0.01;
    }
    else if (fabs(theta2d[0] - thighAngle[0]) < 0.5)
    {
        _t = 0.0;
        stand_ready = true;
    }
}
void Forward_fun()
{
    FR_Forward_Trajectory();
    BR_Forward_Trajectory();
    BL_Forward_Trajectory();
    FL_Forward_Trajectory();
    for (uint8_t i = 0; i < 4; i++)
    {
        Joint2Theta(i);
    }
    n = (int)(1.0 * t / T); // 第0,1,2,.....周期
    t += 0.05;
    GQ_motorSet();
}
void robotStandPosGet()
{
    float first_item[4] = {0, 0, r, -r};  // 初始值
    int sign_item[4] = {1, -1, -1, 1};    // offset.x 的符号
    for (uint8_t i = 0; i < 4; i++)
    {
        xd[i] = first_item[i] + sign_item[i] * offset.x;
        yd[i] = offset.y;
        zd[i] = 0.08;
        Joint2Theta(i);
    }
}
void StandToForward()
{
    if (fabs(offset.z - zd[1]) >= 0.001)
    {
        _t += 0.01;
        for (uint8_t i = 0; i < 4; i++)
        {
            zd[i] = 0.08 + _t * (offset.z - 0.08) / 10.0;
        }
    }
    else if (fabs(offset.z - zd[1]) < 0.001)
    {
        _t = 0;
        spibot_init = true;
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        Joint2Theta(i);
    }
    GQ_motorSet();
}


#endif 