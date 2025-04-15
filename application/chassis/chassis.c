#include "chassis.h"
#include "spibot.h"


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
            .angle_PID = {
                .Kp = 0.1,  // 4.5
                .Ki = 0.05, // 0
                .Kd = 0,    // 0
                .MaxOut = 15000,
                .IntegralLimit = 3000,
            },
            .speed_PID = {
                .Kp = 1, // 4.5
                .Ki = 0, // 0
                .Kd = 0, // 0
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
            .angle_PID = {
                .Kp = 0.1,  // 4.5
                .Ki = 0.05, // 0
                .Kd = 0,    // 0
                .MaxOut = 15000,
                .IntegralLimit = 3000,
            },
            .speed_PID = {
                .Kp = 1, // 4.5
                .Ki = 0, // 0
                .Kd = 0, // 0
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
            .angle_PID = {
                .Kp = 0.1,  // 4.5
                .Ki = 0.05, // 0
                .Kd = 0,    // 0
                .MaxOut = 15000,
                .IntegralLimit = 3000,
            },
            .speed_PID = {
                .Kp = 1, // 4.5
                .Ki = 0, // 0
                .Kd = 0, // 0
                .MaxOut = 15000,
                .IntegralLimit = 3000,
            },
        },
        .motor_type = GQ5047,
    };
    // 高擎注册
    
    angle1_config.can_init_config.tx_id = 1;
    Leg[0].hip = GQMotorInit(&angle1_config);
    angle1_config.can_init_config.tx_id = 2;
    Leg[0].thigh = GQMotorInit(&angle1_config);
    angle1_config.can_init_config.tx_id = 3;
    Leg[0].shank = GQMotorInit(&angle1_config);
    angle1_config.can_init_config.tx_id = 5;
    Leg[1].hip = GQMotorInit(&angle1_config); //引出can1信号接板子

    angle2_config.can_init_config.tx_id = 5;
    Leg[1].thigh = GQMotorInit(&angle2_config);
    angle2_config.can_init_config.tx_id = 6;
    Leg[1].shank = GQMotorInit(&angle2_config); //引出can2信号接板子
    angle2_config.can_init_config.tx_id = 1;
    Leg[2].hip = GQMotorInit(&angle2_config);
    angle2_config.can_init_config.tx_id = 2;
    Leg[2].thigh = GQMotorInit(&angle2_config);

    angle3_config.can_init_config.tx_id = 3;
    Leg[2].shank = GQMotorInit(&angle3_config); //引出can3信号接板子
    angle3_config.can_init_config.tx_id = 4;
    Leg[3].hip = GQMotorInit(&angle3_config);
    angle3_config.can_init_config.tx_id = 5;
    Leg[3].thigh = GQMotorInit(&angle3_config);
    angle3_config.can_init_config.tx_id = 6;
    Leg[3].shank = GQMotorInit(&angle3_config);

    // 计算得到初始角度

    DWT_Delay(0.5);

    
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_SET);

    robotStandPosGet(); 
}



static void SpibotInit()
{
    if (!all_motors_ready)
        motorReadyCheck();// 初始化到12个电机全部接收到信息时大概需要10s
    else{
        if (!stand_ready)
            SpibotStand();
        else
            StandToForward();
    }
}

static void LimitChassisOutput()
{
    if (!spibot_init)
        SpibotInit();   
    else
        Forward_fun();
}

static void thetaToPos()
{
    for (int i = 0; i < 4; i++) 
    {
        rad[3*i + 0] = joint_sign[i][0] * Leg[i].hip->measure.position * toAngle;
        rad[3*i + 1] = joint_sign[i][1] * Leg[i].thigh->measure.position * toAngle;
        rad[3*i + 2] = joint_sign[i][2] * Leg[i].shank->measure.position * toAngle;
        double sin_rad1 = sin(rad[3*i + 1]);      // rad[1], rad[4], rad[7], rad[10]
        double sin_rad2 = sin(rad[3*i + 2]);      // rad[2], rad[5], rad[8], rad[11]
        double cos_rad1 = cos(rad[3*i + 1]);      // rad[1], rad[4], rad[7], rad[10]
        double cos_diff = cos(rad[3*i + 1] - rad[3*i + 2]);  // rad[1]-rad[2], etc.
        double sin_diff = sin(rad[3*i + 1] - rad[3*i + 2]);  // rad[1]-rad[2], etc.
        double term = l2 - l3 * sin_rad2;
        double len = l1 + term * cos_rad1;
        x_pos[i] = sin(rad[3*i]) * len;
        y_pos[i] = cos(rad[3*i]) * len;
        z_pos[i] = l3 * cos_diff - l2 * sin_rad1;
        endV[i].x = -sin(rad[3*i]) * sin_diff;
        endV[i].y = -cos(rad[3*i]) * sin_diff;
        endV[i].z = -cos_diff;
    }
}

void Chassis_task()
{

    thetaToPos();
    // LimitChassisOutput();
}


