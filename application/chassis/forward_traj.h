#ifndef FORWARD_TRAJ_H
#define FORWARD_TRAJ_H

#include "const.h"

void FR_Forward_Trajectory(){
    yd[0] = offset.y; // 沿直线前后摆动
    if (t > 1.0 * n * T && t <= (n + 1 / 6.0) * T)
    {
        xd[0] = 0 + offset.x;
        zd[0] = 0 + offset.z;
    }
    else if ((t > (n + 1 / 6.0) * T) && (t <= (n + 1 / 3.0) * T))
    {
        xd[0] = (-6.0 * r / T) * (t - T / 6.0 - n * T) + offset.x;
        zd[0] = 0 + offset.z;
    }
    else if ((t > (n + 1 / 3.0) * T) && (t <= (n + 1 / 2.0) * T))
    {
        xd[0] = -r + offset.x;
        zd[0] = 0 + offset.z;
    }
    else if ((t > (n + 1 / 2.0) * T) && (t <= (n + 2 / 3.0) * T))
    {
        xd[0] = r * cos(w1 * t) + offset.x;
        yd[0] = offset.y - 0.05 * sin(w1 * t);
        zd[0] = r * sin(w1 * t) + offset.z;
    }
    else if ((t > (n + 2 / 3.0) * T) && (t <= (n + 5 / 6.0) * T))
    {
        xd[0] = (-6.0 * r / T) * (t - 5 * T / 6.0 - n * T) + offset.x;
        zd[0] = 0 + offset.z;
    }
    else if (t > (n + 5 / 6.0) * T && t <= (n + 1.0) * T)
    {
        xd[0] = 0 + offset.x;
        zd[0] = 0 + offset.z;
    }
}
void BR_Forward_Trajectory(){
    yd[1] = offset.y; // 沿直线前后摆动
    if (t > 1.0 * n * T && t <= (n + 1 / 6.0) * T)
    {
        xd[1] = 0 - offset.x;
        zd[1] = 0 + offset.z;
    }
    else if (t > (n + 1 / 6.0) * T && t <= (n + 1 / 3.0) * T)
    {
        xd[1] = (-6.0 * r / T) * (t - T / 6.0 - n * T) - offset.x;
        zd[1] = 0 + offset.z;
    }
    else if (t > (n + 1 / 3.0) * T && t <= (n + 1 / 2.0) * T)
    {
        xd[1] = -r * cos(w1 * t) - offset.x;
        yd[1] = offset.y + 0.05 * sin(w1 * t);
        zd[1] = -r * sin(w1 * t) + offset.z;
    }
    else if (t > (n + 1 / 2.0) * T && t <= (n + 2 / 3.0) * T)
    {
        xd[1] = r - offset.x;
        zd[1] = 0 + offset.z;
    }
    else if (t > (n + 2 / 3.0) * T && t <= (n + 5 / 6.0) * T)
    {
        xd[1] = (-6.0 * r / T) * (t - 5 * T / 6.0 - n * T) - offset.x;
        zd[1] = 0 + offset.z;
    }
    else if (t > (n + 5 / 6.0) * T && t <= (n + 1.0) * T)
    {
        xd[1] = 0 - offset.x;
        zd[1] = 0 + offset.z;
    }
}
void BL_Forward_Trajectory(){
    yd[2] = offset.y; // 沿直线前后摆动
    if (t > 1.0 * n * T && t <= (n + 1 / 6.0) * T)
    {
        xd[2] = r - offset.x;
        zd[2] = 0 + offset.z;
    }
    else if (t > (n + 1 / 6.0) * T && t <= (n + 1 / 3.0) * T)
    {
        xd[2] = (-6.0 * r / T) * (t - T / 3.0 - n * T) - offset.x;
        zd[2] = 0 + offset.z;
    }
    else if (t > (n + 1 / 3.0) * T && t <= (n + 2 / 3.0) * T)
    {
        xd[2] = 0 - offset.x;
        zd[2] = 0 + offset.z;
    }
    else if (t > (n + 2 / 3.0) * T && t <= (n + 5 / 6.0) * T)
    {
        xd[2] = (-6.0 * r / T) * (t - 2 * T / 3.0 - n * T) - offset.x;
        zd[2] = 0 + offset.z;
    }
    else if (t > (n + 5 / 6.0) * T && t <= (n + 1.0) * T)
    {
        xd[2] = r * cos(w1 * t) - offset.x;
        yd[2] = offset.y - 0.05 * sin(w1 * t);
        zd[2] = r * sin(w1 * t) + offset.z;
    }
}
void FL_Forward_Trajectory(){
    yd[3] = offset.y; // 沿直线前后摆动
    if (t > 1.0 * n * T && t <= (n + 1 / 6.0) * T)
    {
        xd[3] = -r * cos(w1 * t) + offset.x;
        yd[3] = offset.y + 0.05 * sin(w1 * t);
        zd[3] = -r * sin(w1 * t) + offset.z;
    }
    else if (t > (n + 1 / 6.0) * T && t <= (n + 1 / 3.0) * T)
    {
        xd[3] = (-6.0 * r / T) * (t - T / 3.0 - n * T) + offset.x;
        zd[3] = 0 + offset.z;
    }
    else if (t > (n + 1 / 3.0) * T && t <= (n + 2 / 3.0) * T)
    {
        xd[3] = 0 + offset.x;
        zd[3] = 0 + offset.z;
    }
    else if (t > (n + 2 / 3.0) * T && t <= (n + 5 / 6.0) * T)
    {
        xd[3] = (-6.0 * r / T) * (t - 2 * T / 3.0 - n * T) + offset.x;
        zd[3] = 0 + offset.z;
    }
    else if (t > (n + 5 / 6.0) * T && t <= (n + 1.0) * T)
    {
        xd[3] = -r + offset.x;
        zd[3] = 0 + offset.z;
    }
}

#endif 