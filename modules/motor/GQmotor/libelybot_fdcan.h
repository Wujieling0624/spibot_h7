// #ifndef _LIBELYBOT_FDCAN_H
// #define _LIBELYBOT_FDCAN_H


// #include "main.h"




// /*
// *   POS_FLAG ???????????????????????????????????????????????????? -0.5?~0.5? ?????????
// *   POS_FLAG = 1  float
// *   POS_FLAG = 2  int32
// *   POS_FLAG = 3  int8
// */
// #define  POS_FLAG  0


// /*
// *   READ_MOTOR_FLAG ???????????????????????
// *   READ_MOTOR_FLAG = 1  float
// *   READ_MOTOR_FLAG = 2  int32
// *   READ_MOTOR_FLAG = 3  int16
// */
// #define  READ_MOTOR_FLAG   3


// /*
// *   POS_REZERO ?????????????????????
// *   ?????????? POS_REZERO ?????????
// *   ?????????? 3 ???????????????
// *   ???????????????????????????????
// */
// // #define  POS_REZERO


// /*
// *   MOTOR_STOP ????????????????
// *   ?????????? MOTOR_STOP ?????????
// *   ?????????? 3 ???????
// *   ??????????????????????? MOTOR_STOP ????????????????????
// */
// // #define  MOTOR_STOP


// /*
// *   MOTOR_BRAKE ????????????????????
// *   ?????????? MOTOR_BRAKE ?????????
// *   ?????????? 3 ????????
// *   ??????????????????????? MOTOR_BRAKE ????????????????????
// */
// // #define  MOTOR_BRAKE


// /* ??????????????????? */
// #define  NAN_FLOAT  NAN
// #define  NAN_INT32  0x80000000
// #define  NAN_INT16  0x8000
// #define  NAN_INT8   0x80



// #define  MODE_POSITION              0X8080
// #define  MODE_VELOCITY              0X8081
// #define  MODE_TORQUE                0X8082
// #define  MODE_VOLTAGE               0X8083
// #define  MODE_CURRENT               0X8084

// #define  MODE_POS_VEL_TQE           0X8090
// #define  MODE_POS_VEL_TQE_KP_KD     0X8093
// #define  MODE_POS_VEL_TQE_KP_KI_KD  0X8098
// #define  MODE_POS_VEL_KP_KD         0X809E
// #define  MODE_POS_VEL_TQE_RKP_RKD   0X80A3
// #define  MODE_POS_VEL_RKP_RKD       0X80A8
// #define  MODE_POS_VEL_ACC           0X80AD



// #if READ_MOTOR_FLAG == 1
// #define MOTOR_SIZE 18
// typedef float motor_state_type;
// #elif READ_MOTOR_FLAG == 2
// #define MOTOR_SIZE 18
// typedef int32_t motor_state_type;
// #elif READ_MOTOR_FLAG == 3
// #define MOTOR_SIZE 10
// typedef int16_t motor_state_type;
// #endif


// typedef struct
// {
//     motor_state_type mode;
//     motor_state_type position;
//     motor_state_type velocity;
//     motor_state_type torque;
//     uint16_t id;
// } motor_state_s;

// typedef struct
// {
//     union
//     {
//         motor_state_s motor;
//         uint8_t data[MOTOR_SIZE];
//     };
// } motor_state_t;


// extern motor_state_t motor_state;
// extern uint8_t motor_read_flag;


// /* dq ????? */
// void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float volt);
// void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t volt);
// void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t volt);

// /* dq ?????? */
// void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float current);
// void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t current);
// void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t current);

// /* ??????? */
// void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque);
// void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque);
// void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque);

// /* ????????????? */
// void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque);
// void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque);
// void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque);

// /* ??? */
// void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos);
// void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos);
// void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos);

// /* ??? */
// void set_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel);
// void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel);
// void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel);

// /* ???????????PD???? */
// void set_pos_vel_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd);
// void set_pos_vel_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd);
// void set_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

// /* ??????????? */
// void set_vel_velmax_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t vel_max);

// /* ???????????????????????? */
// void set_pos_velmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc);
// void set_pos_velmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc);
// void set_pos_velmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc);

// /* ???????????? */
// void set_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc);
// void set_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc);
// void set_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc);

// /* vfoc????? */
// void set_vfoc_lock(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vol);

// /* ???????????????????????(??????????????? 0x17??0x01 ????????????) */
// void timed_return_motor_status_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t_ms);

// /* ???? ?????? */
// void set_many_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t pos2, int16_t pos3, int16_t pos4);

// /* ???? ?????? */
// void set_many_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t vel1, int16_t vel2, int16_t vel3, int16_t vel4);

// /* ???? ??????? */
// void set_many_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t tqe1, int16_t tqe2, int16_t tqe3, int16_t tqe4);

// /* ???? ??????? */
// void set_many_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t volt1, int16_t volt2, int16_t volt3, int16_t volt4);

// /* ???? ???????? */
// void set_many_current_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

// /* ???? ???????????????? */
// void set_many_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t tqe1, int16_t pos2, int16_t vel2, int16_t tqe2);

// /* ???? ??????????????PD???? */
// void set_many_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t tqe1, int16_t rkp1, int16_t rkd1, int16_t pos2, int16_t vel2, int16_t tqe2, int16_t rkp2, int16_t rkd2);

// /* ???? ?????????PD???? */
// void set_many_pos_vel_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t rkp1, int16_t rkd1, int16_t pos2, int16_t vel2, int16_t rkp2, int16_t rkd2);

// /* ???? ????????????????? ?????????? */
// void set_many_pos_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t acc1, int16_t pos2, int16_t vel2, int16_t acc2);

// /* ??????? */
// void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

// /* ???????? */
// void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

// /* ????? */
// void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

// /* ?????? */
// void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

// /* ???????? */
// void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
// void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
// void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);


// #endif
