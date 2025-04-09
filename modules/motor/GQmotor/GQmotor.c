#include "GQmotor.h"
#include "arm_math.h"
#include "string.h"

// #include "bsp_log.h"
#define GQ_MOTOR_CNT 13
#define FRICTION_TORQUE 8.0f//摩擦力矩，发送的时候
static uint8_t idx;

static uint16_t cnt;

static GQMotorInstance *gq_motor_instance[GQ_MOTOR_CNT];
// static osThreadId gq_task_handle[GQ_MOTOR_CNT];
static void GQMotorCaliEncoder(GQMotorInstance *motor);
static void set_conf_write(GQMotorInstance *motor);
void GQMotorGetMeasuretime(GQMotorInstance *motor);
void GQMotor_AngleMode_Setref(GQMotorInstance *motor, float pid_ref);

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
// static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
// {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
// }
// static float uint_to_float(int x_int, float x_min, float x_max, int bits)
// {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
// }

void GQMotorLostCallback(void *motor_ptr)
{
    GQMotorInstance *motor = (GQMotorInstance *)motor_ptr;
    GQMotorGetMeasuretime(motor);
}

/**
 * @brief 设置电机can发送数据
 * @param motor 电机实例
 * @param data 发送数据
 * @param len 数据长度
 */
void GQ_can_set(GQMotorInstance *motor, uint8_t *data, uint8_t len)
{
    motor->motor_can_instace->tx_len = len;
    memcpy(motor->motor_can_instace->tx_buff, data, motor->motor_can_instace->tx_len);
    motor->motor_can_instace->txconf.DataLength = get_fdcan_dlc(motor->motor_can_instace->tx_len);
}

/**
 * @brief 获取电机测量值
 * @param motor 电机实例
 */


void GQMotorGetMeasuretime(GQMotorInstance *motor)
{
    static int16_t t_ms = 5; // 1ms

    const uint8_t cmd[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&cmd[3] = t_ms;
    // 清零tx_buff

    memset(motor->motor_can_instace->tx_buff, 0, sizeof(motor->motor_can_instace->tx_buff));
    memcpy(motor->motor_can_instace->tx_buff, cmd, sizeof(cmd));
    motor->motor_can_instace->txconf.Identifier = 0x8000 | motor->motor_can_instace->tx_id; // 需回复，发送，信号源地址是0,回复数据走decode
    GQ_can_set(motor, cmd, sizeof(cmd));
    CANTransmit(motor->motor_can_instace, 1);

    // DWT_Delay(0.01);

    memset(motor->motor_can_instace->tx_buff, 0, sizeof(motor->motor_can_instace->tx_buff));
    motor->motor_can_instace->txconf.Identifier = 0x0b; // 清空一下免得混乱

    // DWT_Delay(0.1);
    // fdcan_send(&hfdcan1,0x8003,cmd,3);
}

/**
 * @brief 电机解析函数
 * @param motor_can 电机can实例
 */

//浮点数取余数


static void GQMotorDecode(CANInstance *motor_can)
{
    // uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    GQMotorInstance *motor = (GQMotorInstance *)motor_can->id;
    GQ_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);
    if(measure->position - measure->last_position > 1000 || measure->position - measure->last_position < -1000)
    {
        measure->position = measure->last_position;
    }
    else
    {
        measure->last_position = measure->position;
    }
    measure->position = (int16_t)((rxbuff[3] << 8) | rxbuff[2]);
    measure->position = fmod((0.036 * measure->position),360.0); // 协议尺度转化,单位度,单圈
    measure->velocity = (int16_t)((rxbuff[5] << 8) | rxbuff[4])*SPEED_SMOOTH_COEF+measure->velocity*(1-SPEED_SMOOTH_COEF);
    measure->velocity = 360 * (0.00025 * measure->velocity); // 协议尺度转化，单位度/s
    //转化为弧度每秒
    measure->velocity = measure->velocity * 3.1415926 / 180;
    measure->current = (int16_t)((rxbuff[7] << 8) | rxbuff[6])*CURRENT_SMOOTH_COEF+measure->current*(1-CURRENT_SMOOTH_COEF);
    measure->torque = (0.005332 * measure->torque) - 0.072956; // 协议尺度转化，单位N.m

    //多圈角度计算，前提是假设两次采样间电机转过的角度小于180
    if (measure->position - measure->last_position > 180)
        measure->total_round--;
    else if (measure->position - measure->last_position < -180)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->position;



}

/**
 * @brief 初始化电机
 * @param config 电机初始化配置
 * @return GQMotorInstance* 电机实例
 */

GQMotorInstance *GQMotorInit(Motor_Init_Config_s *config)
{
    GQMotorInstance *motor = (GQMotorInstance *)malloc(sizeof(GQMotorInstance));
    memset(motor, 0, sizeof(GQMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;

    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    //使用非电机闭环的时候使用，电机闭环的时候不使用，默认使用电机闭环
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    motor->motor_can_instace->tx_id = config->can_init_config.tx_id; // 记一下id，后面发送的时候再改
    motor->motor_can_instace->rx_id = (config->can_init_config.tx_id << 8);
    motor->motor_close_loop_type = config->controller_setting_init_config.motor_close_loop_type;
    motor->close_loop_type = config->controller_setting_init_config.close_loop_type;
    motor->outer_loop_type = config->controller_setting_init_config.outer_loop_type;
    motor->motor_type = config->motor_type;

    config->can_init_config.rx_id = motor->motor_can_instace->rx_id;
    config->can_init_config.can_module_callback = GQMotorDecode;
    config->can_init_config.id = motor;
    config->can_init_config.can_mode = FD_CAN | EXTID;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = GQMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    // GQMotor_AngleMode_Setref(motor, 0);
    // CANTransmit(motor->motor_can_instace,1);
    // GQMotorEnable(motor);//使能
    // 将新创建的电机实例添加到全局数组中
    gq_motor_instance[idx++] = motor;
    // GQMotorGetMeasuretime(motor);
    DWT_Delay(0.5);
    return motor;
}




void GQMotorEnable(GQMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}




/**
 * @brief 设置电机位置控制
 * @param motor 电机实例
 * @param ref 位置设定值
 *  @param pos 电机1的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 */
void GQMotor_AngleMode_Setref(GQMotorInstance *motor, float pid_ref)
{
    int16_t ref = (int16_t)((pid_ref * 10000) / 360);

    uint8_t tx_data[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x00};
    memcpy(&tx_data[5], &ref, sizeof(int16_t));

    memset(motor->motor_can_instace->tx_buff, 0, sizeof(motor->motor_can_instace->tx_buff));
    memcpy(motor->motor_can_instace->tx_buff, tx_data, sizeof(tx_data));
    motor->motor_can_instace->txconf.Identifier = 0x0000 | motor->motor_can_instace->tx_id; // 需回复，发送，信号源地址是0
    motor->motor_can_instace->tx_len = sizeof(tx_data);
    GQ_can_set(motor, tx_data, motor->motor_can_instace->tx_len);
}

void GQMotor_CurrentMode_Setref(GQMotorInstance *motor, float pid_ref)
{
    int16_t ref;
    ref = (int16_t)(pid_ref);

    // static uint8_t tx_data[] = {0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80,
    //                         //速度      力矩        kp          kd          最大力矩    占位（fdcan）
    //                         0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
    //                        };


    // *(int16_t *)&tx_data[10] = ref;
    // memcpy(motor->motor_can_instace->tx_buff, tx_data, sizeof(tx_data));
    // motor->motor_can_instace->txconf.Identifier = 0x0000 | motor->motor_can_instace->tx_id; // 无需回复，发送，信号源地址是0
    // motor->motor_can_instace->tx_len = sizeof(tx_data);
    // GQ_can_set(motor, tx_data, motor->motor_can_instace->tx_len);


    uint8_t tx_data[8] = {0x05,0x13,0x00,0x80,0x20,0x00,0x80,0x00};
    *(int16_t *)&tx_data[2] = ref;

    memcpy(motor->motor_can_instace->tx_buff, tx_data, sizeof(tx_data));
    motor->motor_can_instace->txconf.Identifier = 0x00+motor->motor_can_instace->tx_id;//无需回复，发送，信号源地址是0
    motor->motor_can_instace->tx_len = sizeof(tx_data);
    GQ_can_set(motor, tx_data, motor->motor_can_instace->tx_len);
}

void GQMotor_Setref(GQMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}


// 使能电机原本是设置stop_flag选择直接设置工作模式，设置为角度


void GQMotorStop(GQMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void GQMotor_Set_Outer_Loop(GQMotorInstance *motor, Closeloop_Type_e outer_loop_type)
{
    motor->outer_loop_type = outer_loop_type;
}

void GQMotor_Set_Motor_Close_Loop(GQMotorInstance *motor, Motor_CloseLoop_Type_e motor_close_loop_type)//设置电机内部闭环还是读到measure后主控闭环
{
    motor->motor_close_loop_type = motor_close_loop_type;
}

/**
 * @brief 设置电机停止,有力气
 * @param motor 电机实例
 */
void set_motor_stop(GQMotorInstance *motor)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00};

    // 设置发送数据
    GQ_can_set(motor, cmd, sizeof(cmd));

    // 使用 CANTransmit 发送
    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief 重置零位
 * @param motor 电机实例
 */
static void GQMotorCaliEncoder(GQMotorInstance *motor)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73, 0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20, 0x30, 0x2e, 0x30, 0x0a};

    // 设置发送数据
    GQ_can_set(motor, cmd, sizeof(cmd));

    // 使用 CANTransmit 发送
    CANTransmit(motor->motor_can_instace, 1);

    DWT_Delay(1);
    set_conf_write(motor);
}

static void set_conf_write(GQMotorInstance *motor)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50};

    // 设置发送数据
    GQ_can_set(motor, cmd, sizeof(cmd));

    // 使用 CANTransmit 发送
    CANTransmit(motor->motor_can_instace, 1);
}

void GQMotor_Zero_force(GQMotorInstance *motor)
{

    GQMotor_CurrentMode_Setref(motor, 0);
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void GQMotorTask()
{
    float pid_measure, pid_ref, set; // 电机PID测量值和设定值
    GQMotorInstance *motor;
    Motor_Controller_s *motor_controller;   // 电机控制器
    GQ_Motor_Measure_s *measure;            // 电机测量值
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    cnt++;
    

    for (size_t i = 0; i < idx; i++)
    {
                motor = gq_motor_instance[i];
        if(motor->stop_flag == MOTOR_STOP)
        {
            GQMotor_Zero_force(motor);
            CANTransmit(gq_motor_instance[i]->motor_can_instace, 1);
            continue;
        }



        if (motor->motor_close_loop_type == MOTOR_CLOSE_LOOP) // 电机内部闭环
        {
            if (motor->outer_loop_type == ANGLE_LOOP)
            {
                GQMotor_AngleMode_Setref(motor, motor->pid_ref);
            }
            else if (motor->outer_loop_type == SPEED_LOOP)
            {
                // GQMotor_SpeedMode_Setref(motor, motor->pid_ref);//暂时没封装,感觉用不上，可以自己封装，很简单
            }
            else if (motor->outer_loop_type == CURRENT_LOOP)
            {
                GQMotor_CurrentMode_Setref(motor, motor->pid_ref);
            }
        }

        else if (motor->motor_close_loop_type == MCU_CLOSE_LOOP) // 读取反馈用单片机做闭环
        {
            motor_setting = &motor->motor_settings;
            motor_controller = &motor->motor_controller;
            measure = &motor->measure;
            pid_ref = motor->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
            if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
            {
                if (motor_setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                else
                    pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃,todo
                // 更新pid_ref进入下一个环
                pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
            }

            // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
            if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            {
                if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                    pid_ref += *motor_controller->speed_feedforward_ptr;

                if (motor_setting->speed_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_speed_feedback_ptr;
                else // MOTOR_FEED
                    pid_measure = measure->velocity;
                // 更新pid_ref进入下一个环
                pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            }

            // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
            if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor_controller->current_feedforward_ptr;
            if (motor_setting->close_loop_type & CURRENT_LOOP)
            {
                pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
            }
            
            if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_ref *= -1;
            
            if(pid_ref > 2)
            {
                GQMotor_CurrentMode_Setref(motor, pid_ref+FRICTION_TORQUE);
            }
            else if(pid_ref < -2)
            {
                GQMotor_CurrentMode_Setref(motor, pid_ref-FRICTION_TORQUE);
            }
            else
            {
                GQMotor_CurrentMode_Setref(motor, 0);
            }
        }
        CANTransmit(gq_motor_instance[i]->motor_can_instace, 1);
    }
    
}

////要去robot_task.c里的OSTaskInit()函数里添加GQMotorControlInit();
// void GQMotorControlInit()
// {
//     char gq_task_name[5] = "gq";
//     // 遍历所有电机实例,创建任务
//     if (!idx)
//         return;
//     for (size_t i = 0; i < idx; i++)
//     {
//         char gq_id_buff[2] = {0};
//         __itoa(i, gq_id_buff, 10);
//         strcat(gq_task_name, gq_id_buff);
//         osThreadDef(gq_task_name, GQMotorTask, osPriorityNormal, 0, 128);
//         gq_task_handle[i] = osThreadCreate(osThread(gq_task_name), gq_motor_instance[i]);
//     }
// }