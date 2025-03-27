#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "dmmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "GQmotor.h"
#include "arm_math.h"
#include "gqmotor.h"
#include "my_fdcan.h"
#include "libelybot_fdcan.h"
#include "super_cap.h"
#include "rm_referee.h"
#include "math.h"
//周期计算，while循环一次时间0.001*TIME_STEP
#define pi 3.14159
#define T 180
#define r 0.15
#define T1 T/3.0
#define T2 T/2.0
#define w1 (2*pi)/(1.0*T1)
#define w2 (2*pi)/(1.0*T2)
#define l1 0.07
#define l2 0.23
#define l3 0.33
#define z_offset 0.2

int n = 0;
float theta1,theta2,theta3,x_offset = 0.2,link_Stretch = 0.23;
float BR_xd,BR_yd,BR_zd;
float FR_xd,FR_yd,FR_zd;
float FL_xd,FL_yd,FL_zd;
float BL_xd,BL_yd,BL_zd;

static GQMotorInstance *fr_thigh,*br_thigh;
static GQMotorInstance *fr_hip,*br_hip;
static GQMotorInstance *fr_shank,*br_shank;

uint16_t t = 0,mt = 0;
void PosToTheta(float xd,float yd,float zd);
void FR_Forward_Trajectory();
void FR_Joint2Theta();
void BR_Forward_Trajectory();
void BR_Joint2Theta();

void ChassisInit()
{
     // 高擎独占can1
      Motor_Init_Config_s Chassis_angle_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_close_loop_type = MOTOR_CLOSE_LOOP,
            .close_loop_type = ANGLE_LOOP,
            .outer_loop_type = ANGLE_LOOP,
        },
        .controller_param_init_config = {
            .angle_PID=
            {
                .Kp = 0.1, // 4.5
                .Ki = 0.05,  // 0
                .Kd = 0,  // 0
                .MaxOut = 15000,
                .IntegralLimit = 3000,
            },
            .speed_PID=
            {
                .Kp = 1, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .MaxOut = 15000,
                .IntegralLimit = 3000,
            },
        },
        .motor_type = GQ5047,
    };
    
    //高擎注册
    Chassis_angle_config.can_init_config.tx_id = 1;
    fr_hip = GQMotorInit(&Chassis_angle_config);
    Chassis_angle_config.can_init_config.tx_id = 2;
    fr_thigh = GQMotorInit(&Chassis_angle_config);
    Chassis_angle_config.can_init_config.tx_id = 3;
    fr_shank = GQMotorInit(&Chassis_angle_config);
    Chassis_angle_config.can_init_config.tx_id = 4;
    br_hip = GQMotorInit(&Chassis_angle_config);
    Chassis_angle_config.can_init_config.tx_id = 5;
    br_thigh = GQMotorInit(&Chassis_angle_config); 
    Chassis_angle_config.can_init_config.tx_id = 6;
    br_shank = GQMotorInit(&Chassis_angle_config); 
    GQMotorEnable(fr_hip); GQMotorEnable(br_hip); 
    GQMotorEnable(fr_thigh); GQMotorEnable(br_thigh); 
    GQMotorEnable(fr_shank); GQMotorEnable(br_shank); 
}


static void LimitChassisOutput()
{
    // FR_Forward_Trajectory();
    BR_Forward_Trajectory();
    // FR_Joint2Theta();
    BR_Joint2Theta();
    mt++;
    if(mt%10 == 0)
    {
        n = (int)(1.0*t/T);  //第0,1,2,.....周期
        t++;
    }
}

void  Chassis_task()
{
    LimitChassisOutput();
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
  if ((t>(n+1/6.0)*T) && (t<=(n+1/3.0)*T)){
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
  else {
    FR_xd = 0+x_offset;
    FR_zd = 0+z_offset;
  }   
}
void BR_Forward_Trajectory()
{
  BR_yd = link_Stretch;  //沿直线前后摆动
  if (t>(n+1/6.0)*T && t<=(n+1/3.0)*T){
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
  else {
    BR_xd = 0-x_offset;
    BR_zd = 0+z_offset;  
  }
}

void FR_Joint2Theta()
{
    PosToTheta(FR_xd,FR_yd,FR_zd);
    float theta1d = theta1;   //theta1目标位置 
    float theta2d = -theta2;   //theta2目标位置 
    float theta3d = -theta3;   //theta3目标位置 
    GQMotor_Setref(fr_hip,theta1d);
    GQMotor_Setref(fr_thigh,theta2d);
    GQMotor_Setref(fr_shank,theta3d);
}
void BR_Joint2Theta()
{
    PosToTheta(BR_xd,BR_yd,BR_zd);
    float theta1d = theta1;   //theta1目标位置 
    float theta2d = theta2;   //theta2目标位置 
    float theta3d = theta3;   //theta3目标位置 
    GQMotor_Setref(br_hip,theta1d);
    GQMotor_Setref(br_thigh,theta2d);
    GQMotor_Setref(br_shank,theta3d);
}