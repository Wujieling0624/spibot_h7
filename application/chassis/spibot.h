#ifndef SPIBOT_H
#define SPIBOT_H

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
            Leg[i].legMotorReady = true;
        }
    }
}

static void thetaToPos()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        rad[3 * i + 0] = joint_sign[i][0] * Leg[i].hip->measure.position * toRad;
        rad[3 * i + 1] = joint_sign[i][1] * Leg[i].thigh->measure.position * toRad;
        rad[3 * i + 2] = joint_sign[i][2] * Leg[i].shank->measure.position * toRad;

        double sin_rad0 = sin(rad[3 * i + 0]);                  // rad[0], rad[3], rad[6], rad[9]
        double sin_rad1 = sin(rad[3 * i + 1]);                  // rad[1], rad[4], rad[7], rad[10]
        double sin_rad2 = sin(rad[3 * i + 2]);                  // rad[2], rad[5], rad[8], rad[11]
        double cos_rad0 = cos(rad[3 * i + 0]);                  // rad[0], rad[3], rad[6], rad[9]
        double cos_rad1 = cos(rad[3 * i + 1]);                  // rad[1], rad[4], rad[7], rad[10]
        double cos_diff = cos(rad[3 * i + 1] - rad[3 * i + 2]); // rad[1]-rad[2], etc.
        double sin_diff = sin(rad[3 * i + 1] - rad[3 * i + 2]); // rad[1]-rad[2], etc.
        Leg[i].endEffector_posX = l3 * sin_rad0 * sin_diff + l2 * sin_rad0 * cos_rad1 + l1 * sin_rad0;
        Leg[i].endEffector_posY = l3 * cos_rad0 * sin_diff + l2 * cos_rad0 * cos_rad1 + l1 * cos_rad0;
        Leg[i].endEffector_posZ = l3 * cos_diff - l2 * sin_rad1;

        Leg[i].endV.x = -sin(rad[3 * i]) * sin_diff;
        Leg[i].endV.y = -cos(rad[3 * i]) * sin_diff;
        Leg[i].endV.z = -cos_diff;
    }
}

void PosToTheta(double xd, double yd, double zd, uint8_t leg_id)
{
    double L = sqrt(pow(xd, 2) + pow(yd, 2));
    rad[3 * leg_id + 0] = atan2(xd, yd); // 弧度
    double ar = atan2(zd, L - l1);       // 弧度
    double Lr = sqrt(pow(zd, 2) + pow((L - l1), 2));
    double a1 = acos((pow(Lr, 2) + pow(l2, 2) - pow(l3, 2)) / (2.0 * l2 * Lr)); // 弧度
    double a2 = acos((pow(Lr, 2) + pow(l3, 2) - pow(l2, 2)) / (2.0 * Lr * l3)); // 弧度
    rad[3 * leg_id + 1] = a1 - ar;                                              // 弧度
    rad[3 * leg_id + 2] = -90 / 180.0 * pi + a1 + a2;                           // 弧度
    theta1[leg_id] = rad[3 * leg_id + 0] * toAngle;
    theta2[leg_id] = rad[3 * leg_id + 1] * toAngle;
    theta3[leg_id] = rad[3 * leg_id + 2] * toAngle;
}

void motorReadyCheck()
{
    InitAngleSet();
    if (Leg[0].legMotorReady && Leg[1].legMotorReady && Leg[2].legMotorReady && Leg[3].legMotorReady)
    {
        all_motors_ready = true;
    }
}

void XYZ2Theta(uint8_t leg_id)
{
    PosToTheta(xd[leg_id], yd[leg_id], zd[leg_id], leg_id);
    theta1d[leg_id] = joint_sign[leg_id][0] * theta1[leg_id];
    theta2d[leg_id] = joint_sign[leg_id][1] * theta2[leg_id];
    theta3d[leg_id] = joint_sign[leg_id][2] * theta3[leg_id];
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
        XYZ2Theta(i);
    n = (int)(1.0 * _t / T); // 第0,1,2,.....周期
    _t += 0.01;
    GQ_motorSet();
}

// 作用：机体后退move_x米，向上move_z米，从B往F看是向左move_y米，除去摆动相
void baseToXYZ(double move_x, double move_y, double move_z)
{
    int ysign[4] = {1, 1, -1, -1}; // 0:fr 1:br 2:bl 3:fl
    int SwingIndex = -1;
    uint8_t SupportIndex = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (Leg[i].swingPhase == true)
        {
            SwingIndex = i;
            break;
        }
        else
        {
            SupportIndex = i;
        }
    }
    if (fabs(Leg[SupportIndex].endEffector_posX + move_x - xd[SupportIndex]) >= 0.001)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            if (SwingIndex == i)
            {
                xd[i] = xd[i];
                yd[i] = yd[i];
                zd[i] = zd[i];
            }
            else
            {
                xd[i] = Leg[i].endEffector_posX + _t * move_x / 10.0;
                yd[i] = Leg[i].endEffector_posY + _t * ysign[i] * move_y / 10.0;
                zd[i] = Leg[i].endEffector_posZ + _t * move_z / 10.0;
            }
        }
        _t += 0.05;
    }
    else if (fabs(Leg[SupportIndex].endEffector_posX + move_x - xd[SupportIndex]) < 0.001)
    {
        _t = 0;
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        XYZ2Theta(i);
    }
    GQ_motorSet();
}

void robotStandPosGet()
{
    // float first_item[4] = {0, 0, r, -r};  // 前进步态初始值
    float first_item[4] = {0, 0, 0, -0}; // 前进步态初始值
    int sign_item[4] = {1, -1, -1, 1};   // offset.x 的符号
    for (uint8_t i = 0; i < 4; i++)
    {
        xd[i] = first_item[i] + sign_item[i] * offset.x;
        yd[i] = offset.y;
        zd[i] = 0.08;
        XYZ2Theta(i);
    }
}

void StandToForward()
{
    if (fabs(offset.z - zd[1]) >= 0.001)
    {
        _t += 0.01;
        for (uint8_t i = 0; i < 4; i++)
        {
            xd[i] = xd[i];
            yd[i] = yd[i];
            zd[i] = 0.08 + _t * (offset.z - 0.08) / 10.0;
            XYZ2Theta(i);
        }
    }
    else if (fabs(offset.z - zd[1]) < 0.001)
    {
        _t = 0;
        spibot_init = true;
    }
    GQ_motorSet();
}

#endif