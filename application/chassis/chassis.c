#include "chassis.h"
#include "spibot.h"


void PosToTheta(float xd,float yd,float zd);
void FR_Forward_Trajectory(); 
void BR_Forward_Trajectory(); 
void BL_Forward_Trajectory(); 
void FL_Forward_Trajectory();
void FR_Joint2Theta(); 
void BR_Joint2Theta(); 
void BL_Joint2Theta(); 
void FL_Joint2Theta();
void GQ_motorSet();
void Forward_fun();
void robotStandPos();

void ChassisInit()
{
  // 高擎独占can1\2\3
  Motor_Init_Config_s angle1_config = {
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

  Motor_Init_Config_s angle2_config = {
    .can_init_config = {
        .can_handle = &hcan2,
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

  Motor_Init_Config_s angle3_config = {
    .can_init_config = {
        .can_handle = &hcan3,
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
  angle1_config.can_init_config.tx_id = 1;
  fr_hip = GQMotorInit(&angle1_config);
  angle1_config.can_init_config.tx_id = 2;
  fr_thigh = GQMotorInit(&angle1_config);
  angle1_config.can_init_config.tx_id = 3;
  fr_shank = GQMotorInit(&angle1_config);
  angle1_config.can_init_config.tx_id = 4;
  br_hip = GQMotorInit(&angle1_config);

  angle2_config.can_init_config.tx_id = 5;
  br_thigh = GQMotorInit(&angle2_config); 
  angle2_config.can_init_config.tx_id = 6;
  br_shank = GQMotorInit(&angle2_config); 
  angle2_config.can_init_config.tx_id = 1;
  bl_hip = GQMotorInit(&angle2_config); 
  angle2_config.can_init_config.tx_id = 2;
  bl_thigh = GQMotorInit(&angle2_config); 

  angle3_config.can_init_config.tx_id = 3;
  bl_shank = GQMotorInit(&angle3_config); 
  angle3_config.can_init_config.tx_id = 4;
  fl_hip = GQMotorInit(&angle3_config); 
  angle3_config.can_init_config.tx_id = 5;
  fl_thigh = GQMotorInit(&angle3_config); 
  angle3_config.can_init_config.tx_id = 6;
  fl_shank = GQMotorInit(&angle3_config); 
  // 计算得到初始角度

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin|POWER_24V_1_Pin, GPIO_PIN_SET);

  FR_Forward_Trajectory(); BR_Forward_Trajectory(); BL_Forward_Trajectory(); FL_Forward_Trajectory();
  FR_Joint2Theta(); BR_Joint2Theta(); BL_Joint2Theta(); FL_Joint2Theta();

}

static void LimitChassisOutput()
{
  if (!motor_ready){
    frInitAngleSet();
    brInitAngleSet();
    blInitAngleSet();
    flInitAngleSet();
    if (brmotor && frmotor && flmotor && blmotor)
    {
      motor_ready = true;
    }
  }
  else if (motor_ready){
    if (!init_flag)
    {
      if(fabs(FR_theta1d - fr_hipangle) >= 1.0)
      {
        fl_hipangle = flinit_hipangle + _t*(FL_theta1d-flinit_hipangle)/5.0;
        fl_thighangle = flinit_thighangle + _t*(FL_theta2d-flinit_thighangle)/5.0;
        fl_shankangle = flinit_shankangle + _t*(FL_theta3d-flinit_shankangle)/5.0;
        bl_hipangle = blinit_hipangle + _t*(BL_theta1d-blinit_hipangle)/5.0;
        bl_thighangle = blinit_thighangle + _t*(BL_theta2d-blinit_thighangle)/5.0;
        bl_shankangle = blinit_shankangle + _t*(BL_theta3d-blinit_shankangle)/5.0;
        fr_hipangle = frinit_hipangle + _t*(FR_theta1d-frinit_hipangle)/5.0;
        fr_thighangle = frinit_thighangle + _t*(FR_theta2d-frinit_thighangle)/5.0;
        fr_shankangle = frinit_shankangle + _t*(FR_theta3d-frinit_shankangle)/5.0;
        br_hipangle = brinit_hipangle + _t*(BR_theta1d-brinit_hipangle)/5.0;
        br_thighangle = brinit_thighangle + _t*(BR_theta2d-brinit_thighangle)/5.0;
        br_shankangle = brinit_shankangle + _t*(BR_theta3d-brinit_shankangle)/5.0;
        _t+=0.02;
        GQMotor_Setref(fl_hip,fl_hipangle); GQMotor_Setref(fl_thigh,fl_thighangle); GQMotor_Setref(fl_shank,fl_shankangle);
        GQMotor_Setref(bl_hip,bl_hipangle); GQMotor_Setref(bl_thigh,bl_thighangle); GQMotor_Setref(bl_shank,bl_shankangle);
        GQMotor_Setref(fr_hip,fr_hipangle); GQMotor_Setref(fr_thigh,fr_thighangle); GQMotor_Setref(fr_shank,fr_shankangle);
        GQMotor_Setref(br_hip,br_hipangle); GQMotor_Setref(br_thigh,br_thighangle); GQMotor_Setref(br_shank,br_shankangle);
      } 
      else if (abs(FR_theta1d - fr_hipangle) < 1.0)
      {
    //     init_flag = true;
    //     hip_ready = true;
      }

      // }

    }
    // else if (init_flag)
    // {
    //     Forward_fun();
    // }
  }
}

void  Chassis_task()
{
    LimitChassisOutput();
}


void robotStandPos()
{


}
