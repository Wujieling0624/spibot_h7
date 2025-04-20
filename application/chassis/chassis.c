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

void SpibotInit()
{
    if (!all_motors_ready)
        motorReadyCheck(); // 初始化到12个电机全部接收到信息时大概需要10s
    else{
        if (!stand_ready)
            SpibotStand(); // 此时_t = 0
        else
            StandToForward(); // 此时_t = 0
    }
}

void getPos_once(void) {
    static bool initialized = false;  // 静态变量，只在第一次初始化为false
    if (!initialized) {
        thetaToPos(); // 有了endEffector_posX,Y,Z
        initialized = true;  // 标记为已初始化
    }
}


 
void LimitChassisOutput()
{
    if (!spibot_init)
    {
        SpibotInit(); 
    }
    else
    {
        // baseToXYZ(0.1,0.1,0.0);
        baseToRPY(10, 10, 10);
    }
}

double rad_diff;
double theta_diff;
// 法向量到三个关节的角度不是一一对应关系，可以加上对zd的要求使得theta2和theta3确定唯一值。
// endVToTheta(Leg[i].endV.x, Leg[i].endV.y, Leg[i].endV.z, Leg[i].endEffector_posZ, i); 
// bug：需要在一定的角度范围内，不然会+-180突变
void endVToTheta(double Vx, double Vy, double Vz, double zd, uint8_t leg_id)
{
    rad[3 * leg_id + 0] = atan(Vx / Vy); // 弧度
    rad[3 * leg_id + 1] = asin(-(zd + l3 * Vz) / l2);
    rad_diff = atan2(sqrt(pow(Vx, 2) + pow(Vy, 2)), Vz);
    theta_diff = rad_diff * toAngle;
    rad[3 * leg_id + 2] = rad[3 * leg_id + 1] - rad_diff + M_PI;
    
    theta1[leg_id] = rad[3 * leg_id + 0] * toAngle;
    theta2[leg_id] = rad[3 * leg_id + 1] * toAngle;
    theta3[leg_id] = rad[3 * leg_id + 2] * toAngle;
    theta1d[leg_id] = joint_sign[leg_id][0] * theta1[leg_id];
    theta2d[leg_id] = joint_sign[leg_id][1] * theta2[leg_id];
    theta3d[leg_id] = joint_sign[leg_id][2] * theta3[leg_id];
}


void Chassis_task()
{
    // thetaToPos();
    // for (uint8_t i = 0; i < 4; i++) 
    // {
    // }   
    LimitChassisOutput();

}

