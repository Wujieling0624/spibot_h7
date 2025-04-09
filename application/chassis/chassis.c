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
    fr_hip = GQMotorInit(&angle1_config);
    angle1_config.can_init_config.tx_id = 2;
    fr_thigh = GQMotorInit(&angle1_config);
    angle1_config.can_init_config.tx_id = 3;
    fr_shank = GQMotorInit(&angle1_config);
    angle1_config.can_init_config.tx_id = 5;
    br_hip = GQMotorInit(&angle1_config); //引出can1信号接板子

    angle2_config.can_init_config.tx_id = 5;
    br_thigh = GQMotorInit(&angle2_config);
    angle2_config.can_init_config.tx_id = 6;
    br_shank = GQMotorInit(&angle2_config); //引出can2信号接板子
    angle2_config.can_init_config.tx_id = 1;
    bl_hip = GQMotorInit(&angle2_config);
    angle2_config.can_init_config.tx_id = 2;
    bl_thigh = GQMotorInit(&angle2_config);

    angle3_config.can_init_config.tx_id = 3;
    bl_shank = GQMotorInit(&angle3_config); //引出can3信号接板子
    angle3_config.can_init_config.tx_id = 4;
    fl_hip = GQMotorInit(&angle3_config);
    angle3_config.can_init_config.tx_id = 5;
    fl_thigh = GQMotorInit(&angle3_config);
    angle3_config.can_init_config.tx_id = 6;
    fl_shank = GQMotorInit(&angle3_config);

    // 计算得到初始角度

    DWT_Delay(1.0);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOC, POWER_24V_2_Pin | POWER_24V_1_Pin, GPIO_PIN_SET);

    robotStandPosGet(); //已经有了
}

static void LimitChassisOutput()
{
    if (!motor_ready)
    {
        motorReadyCheck();
    }
    else if (motor_ready)
    {
        if (!stand_ready)
        {
            SpibotStand();
        }
        else if (stand_ready)
        {
            StandToForward();
        }
        // else if (init_flag)
        // {
        //     Forward_fun();
        // }
    }
}

void Chassis_task()
{
    LimitChassisOutput();
}
