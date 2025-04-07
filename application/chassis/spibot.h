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
#include "math.h"
#include <stdbool.h>
#include "main.h"


//周期计算，while循环一次时间0.001*TIME_STEP
#define pi 3.14159
#define T 120
#define r 0.15
#define T1 T/3.0
#define T2 T/2.0
#define w1 (2*pi)/(1.0*T1)
#define w2 (2*pi)/(1.0*T2)
#define l1 0.07
#define l2 0.23
#define l3 0.33
#define z_offset 0.25

static GQMotorInstance *fr_hip, *fr_thigh, *fr_shank;
static GQMotorInstance *br_hip, *br_thigh, *br_shank;
static GQMotorInstance *bl_hip, *bl_thigh, *bl_shank;
static GQMotorInstance *fl_hip, *fl_thigh, *fl_shank;

int n = 0;
float theta1,theta2,theta3,x_offset = 0.2,link_Stretch = 0.18;
float BR_xd,BR_yd,BR_zd,BR_theta1d,BR_theta2d,BR_theta3d;
float FR_xd,FR_yd,FR_zd,FR_theta1d,FR_theta2d,FR_theta3d;
float FL_xd,FL_yd,FL_zd,FL_theta1d,FL_theta2d,FL_theta3d;
float BL_xd,BL_yd,BL_zd,BL_theta1d,BL_theta2d,BL_theta3d;

uint16_t mt = 0, _mt = 0;;
float t = 0.000001, _t = 0.000001;
bool init_flag = false, motor_ready = false, frmotor = false, brmotor = false, blmotor = false, flmotor = false;
bool hip_ready = false, thigh_ready = false, shank_ready = false;
float flinit_hipangle = 0.0, flinit_thighangle = 0.0, flinit_shankangle = 0.0;
float blinit_hipangle = 0.0, blinit_thighangle = -0.0, blinit_shankangle = -0.0;
float frinit_hipangle = 0.0, frinit_thighangle = -0.0, frinit_shankangle = -0.0;
float brinit_hipangle = 0.0, brinit_thighangle = 0.0, brinit_shankangle = 0.0;
float fl_hipangle = 0.0, fl_thighangle = 0.0, fl_shankangle = 0.0;
float bl_hipangle = 0.0, bl_thighangle = 0.0, bl_shankangle = 0.0;
float fr_hipangle = 0.0, fr_thighangle = 0.0, fr_shankangle = 0.0;
float br_hipangle = 0.0, br_thighangle = 0.0, br_shankangle = 0.0;


void frInitAngleSet()
{
 if (fr_hip->measure.current != 0 && fr_thigh->measure.current != 0 && fr_shank->measure.current != 0)
 {
    GQMotorEnable(fr_hip); GQMotorEnable(fr_thigh); GQMotorEnable(fr_shank); 
    frinit_hipangle = fr_hip->measure.position;
    frinit_thighangle = fr_thigh->measure.position;
    frinit_shankangle = fr_shank->measure.position;
    GQMotor_Setref(fr_hip,frinit_hipangle); GQMotor_Setref(fr_thigh,frinit_thighangle); GQMotor_Setref(fr_shank,frinit_shankangle);
    frmotor = true;
 }
}
void brInitAngleSet()
{
  if (br_hip->measure.current != 0 && br_thigh->measure.current != 0 && br_shank->measure.current != 0)
  {
    GQMotorEnable(br_hip); GQMotorEnable(br_thigh); GQMotorEnable(br_shank); 
    brinit_hipangle = br_hip->measure.position;
    brinit_thighangle = br_thigh->measure.position;
    brinit_shankangle = br_shank->measure.position;
    GQMotor_Setref(br_hip,brinit_hipangle); GQMotor_Setref(br_thigh,brinit_thighangle); GQMotor_Setref(br_shank,brinit_shankangle);
    brmotor = true;
  }
}
void blInitAngleSet()
{
  if (bl_hip->measure.current != 0 && bl_thigh->measure.current != 0 && bl_shank->measure.current != 0)
  {
    GQMotorEnable(bl_hip); GQMotorEnable(bl_thigh); GQMotorEnable(bl_shank); 
    blinit_hipangle = bl_hip->measure.position;
    blinit_thighangle = bl_thigh->measure.position;
    blinit_shankangle = bl_shank->measure.position;
    GQMotor_Setref(bl_hip,blinit_hipangle); GQMotor_Setref(bl_thigh,blinit_thighangle); GQMotor_Setref(bl_shank,blinit_shankangle);
    blmotor = true;
  }
}
void flInitAngleSet()
{
  if (fl_hip->measure.current != 0 && fl_thigh->measure.current != 0 && fl_shank->measure.current != 0)
  {
    GQMotorEnable(fl_hip); GQMotorEnable(fl_thigh); GQMotorEnable(fl_shank); 
    flinit_hipangle = fl_hip->measure.position;
    flinit_thighangle = fl_thigh->measure.position;
    flinit_shankangle = fl_shank->measure.position;
    GQMotor_Setref(fl_hip,flinit_hipangle); GQMotor_Setref(fl_thigh,flinit_thighangle); GQMotor_Setref(fl_shank,flinit_shankangle);
    flmotor = true;
  }
}

void PosToTheta(float xd,float yd,float zd)
{
  float L = sqrt(pow(xd,2)+pow(yd,2)); 
  theta1 = atan2(xd,yd);  //弧度 
  float ar = atan2(zd,L-l1);  //弧度 
  float Lr = sqrt(pow(zd,2)+pow((L-l1),2)); 
  float a1 = acos((pow(Lr,2)+pow(l2,2)-pow(l3,2))/(2.0*l2*Lr));  //弧度 
  float a2 = acos((pow(Lr,2)+pow(l3,2)-pow(l2,2))/(2.0*Lr*l3));  //弧度 
  theta2 = a1-ar;   //弧度 
  theta3 = -90/180.0*pi+a1+a2;  //弧度  
  theta1 = (theta1/pi)*180;
  theta2 = (theta2/pi)*180;
  theta3 = (theta3/pi)*180;
}


void FR_Forward_Trajectory()
{
  FR_yd = link_Stretch;  //沿直线前后摆动
  if (t>1.0*n*T && t<=(n+1/6.0)*T){
    FR_xd = 0+x_offset;
    FR_zd = 0+z_offset;
  } 
  else if ((t>(n+1/6.0)*T) && (t<=(n+1/3.0)*T)){
    FR_xd = (-6.0*r/T)*(t-T/6.0-n*T)+x_offset;
    FR_zd = 0+z_offset;
  }
  else if ((t>(n+1/3.0)*T) && (t<=(n+1/2.0)*T)){
    FR_xd = -r+x_offset;
    FR_zd = 0+z_offset;
  }
  else if ((t>(n+1/2.0)*T) && (t<=(n+2/3.0)*T)){
    FR_xd = r*cos(w1*t)+x_offset;
    FR_yd = link_Stretch-0.05*sin(w1*t);
    FR_zd = r*sin(w1*t)+z_offset;
  }
  else if ((t>(n+2/3.0)*T) && (t<=(n+5/6.0)*T)){
    FR_xd = (-6.0*r/T)*(t-5*T/6.0-n*T)+x_offset;
    FR_zd = 0+z_offset;
  }
  else if (t>(n+5/6.0)*T && t<=(n+1.0)*T){
    FR_xd = 0+x_offset;
    FR_zd = 0+z_offset;
  }   
}
void BR_Forward_Trajectory()
{
  BR_yd = link_Stretch;  //沿直线前后摆动
  if (t>1.0*n*T && t<=(n+1/6.0)*T){
    BR_xd = 0-x_offset;  
    BR_zd = 0+z_offset;  
  }
  else if (t>(n+1/6.0)*T && t<=(n+1/3.0)*T){
    BR_xd = (-6.0*r/T)*(t-T/6.0-n*T)-x_offset;
    BR_zd = 0+z_offset;
  }
  else if (t>(n+1/3.0)*T && t<=(n+1/2.0)*T){
    BR_xd = -r*cos(w1*t)-x_offset;
    BR_yd = link_Stretch+0.05*sin(w1*t);
    BR_zd = -r*sin(w1*t)+z_offset;  
  }
  else if (t>(n+1/2.0)*T && t<=(n+2/3.0)*T){
    BR_xd = r-x_offset;
    BR_zd = 0+z_offset;  
  }
  else if (t>(n+2/3.0)*T && t<=(n+5/6.0)*T){
    BR_xd = (-6.0*r/T)*(t-5*T/6.0-n*T)-x_offset;
    BR_zd = 0+z_offset; 
  }
  else if (t>(n+5/6.0)*T && t<=(n+1.0)*T){
    BR_xd = 0-x_offset;
    BR_zd = 0+z_offset;  
  }
}
void BL_Forward_Trajectory()
{
  BL_yd = link_Stretch;  //沿直线前后摆动
  if (t>1.0*n*T && t<=(n+1/6.0)*T){
    BL_xd = r-x_offset;
    BL_zd = 0+z_offset;
  }
  else if (t>(n+1/6.0)*T && t<=(n+1/3.0)*T){
    BL_xd = (-6.0*r/T)*(t-T/3.0-n*T)-x_offset;
    BL_zd = 0+z_offset;  
  }
  else if (t>(n+1/3.0)*T && t<=(n+2/3.0)*T){
    BL_xd = 0-x_offset;
    BL_zd = 0+z_offset;
  }
  else if (t>(n+2/3.0)*T && t<=(n+5/6.0)*T){
    BL_xd = (-6.0*r/T)*(t-2*T/3.0-n*T)-x_offset;
    BL_zd = 0+z_offset;  
  }
  else if (t>(n+5/6.0)*T && t<=(n+1.0)*T){
    BL_xd = r*cos(w1*t)-x_offset;
    BL_yd = link_Stretch-0.05*sin(w1*t);
    BL_zd = r*sin(w1*t)+z_offset;
  }
}
void FL_Forward_Trajectory()
{
  FL_yd = link_Stretch;  //沿直线前后摆动
  if (t>1.0*n*T && t<=(n+1/6.0)*T){
    FL_xd = -r*cos(w1*t)+x_offset;
    FL_yd = link_Stretch+0.05*sin(w1*t);
    FL_zd = -r*sin(w1*t)+z_offset;  
  }
  else if (t>(n+1/6.0)*T && t<=(n+1/3.0)*T){
    FL_xd = (-6.0*r/T)*(t-T/3.0-n*T)+x_offset;
    FL_zd = 0+z_offset;  
  }
  else if (t>(n+1/3.0)*T && t<=(n+2/3.0)*T){
    FL_xd = 0+x_offset;
    FL_zd = 0+z_offset;
  }
  else if (t>(n+2/3.0)*T && t<=(n+5/6.0)*T){
    FL_xd = (-6.0*r/T)*(t-2*T/3.0-n*T)+x_offset;
    FL_zd = 0+z_offset;  
  }
  else if (t>(n+5/6.0)*T && t<=(n+1.0)*T){
    FL_xd = -r+x_offset;
    FL_zd = 0+z_offset;  
  }
}

void FR_Joint2Theta()
{
  PosToTheta(FR_xd,FR_yd,FR_zd);
  FR_theta1d = theta1;   //theta1目标位置 
  FR_theta2d = -theta2;   //theta2目标位置 
  FR_theta3d = -theta3;   //theta3目标位置 
}
void BR_Joint2Theta()
{
  PosToTheta(BR_xd,BR_yd,BR_zd);
  BR_theta1d = theta1;   //theta1目标位置 
  BR_theta2d = theta2;   //theta2目标位置 
  BR_theta3d = theta3;   //theta3目标位置 
}
void BL_Joint2Theta()
{
  PosToTheta(BL_xd,BL_yd,BL_zd);
  BL_theta1d = -theta1;   //theta1目标位置 
  BL_theta2d = -theta2;   //theta2目标位置 
  BL_theta3d = -theta3;   //theta3目标位置 
}
void FL_Joint2Theta()
{
  PosToTheta(FL_xd,FL_yd,FL_zd);
  FL_theta1d = -theta1;   //theta1目标位置 
  FL_theta2d = theta2;   //theta2目标位置 
  FL_theta3d = theta3;   //theta3目标位置 
}

void GQ_motorSet()
{
  GQMotor_Setref(fr_hip,FR_theta1d);
  GQMotor_Setref(fr_thigh,FR_theta2d);
  GQMotor_Setref(fr_shank,FR_theta3d);
  GQMotor_Setref(br_hip,BR_theta1d);
  GQMotor_Setref(br_thigh,BR_theta2d);
  GQMotor_Setref(br_shank,BR_theta3d);
  GQMotor_Setref(bl_hip,BL_theta1d);
  GQMotor_Setref(bl_thigh,BL_theta2d);
  GQMotor_Setref(bl_shank,BL_theta3d);
  GQMotor_Setref(fl_hip,FL_theta1d);
  GQMotor_Setref(fl_thigh,FL_theta2d);
  GQMotor_Setref(fl_shank,FL_theta3d);
}



void Forward_fun()
{
    mt++;
    if(mt%5 == 0)
    {
      FR_Forward_Trajectory();
      BR_Forward_Trajectory();
      BL_Forward_Trajectory();
      FL_Forward_Trajectory();
      FR_Joint2Theta();
      BR_Joint2Theta();
      BL_Joint2Theta();
      FL_Joint2Theta();
      n = (int)(1.0*t/T);  //第0,1,2,.....周期
      t+=0.2;
    } 
    GQ_motorSet();  
}


#endif // CHASSIS_H