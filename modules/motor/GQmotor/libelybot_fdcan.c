// #include "livelybot_fdcan.h"
// #include "my_fdcan.h"
// #include <string.h>


// FDCAN_RxHeaderTypeDef fdcan_rx_header1;
// uint8_t fdcan1_rdata[64] = {0};

// motor_state_t motor_state;
// uint8_t motor_read_flag = 0;



// //static void print_data(uint8_t *data, uint16_t len)
// //{
// //    for (int i = 0; i < len; i++)
// //    {
// //        printf("%d\r\n", &data[i]);
// //    }
// //    printf("\r\n\r\n");
// //}


// /**
//  * @brief 电压控制 float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param volt 电压，例：0.3 -> 0.3v
//  */
// void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float volt)
// {
//     //                          dq电压模式   2个16位      d                        q                       占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0E, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     *(float *)&cmd[9] = volt;

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电压控制 int32
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param volt 电压，单位：0.001V
//  */
// void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t volt)
// {
//     //                   		dq电压模式   2个32位      d                        q                       占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0A, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     *(int32_t *)&cmd[9] = volt;

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电压控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param volt 电压，单位：0.1V
//  */
// void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t volt)
// {
//     //                   		dq电压模式   2个16位      d           q           占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x06, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     *(int16_t *)&cmd[7] = volt;

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电流控制 float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param volt 电流，例：0.3 -> 0.3A
//  */
// void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float current)
// {
//     //                   		dq电流模式   2个32位      q电流       			  d电流       			  占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     *(float *)&cmd[5] = current;

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电流控制 int32
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param volt 电流，单位：0.001A
//  */
// void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t current)
// {
//     //                   		dq电流模式   2个32位      q电流       			  d电流       			   占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0A, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     *(int32_t *)&cmd[5] = current;

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电流控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param volt 电流，单位：0.1A
//  */
// void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t current)
// {
//     //                   		dq电流模式   2个16位      q电流       d电流       占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x06, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     *(int16_t *)&cmd[5] = current;

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 力矩控制 float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param torque 力矩（单位见文档）
//  */
// void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque)
// {
//     //                     		  位置模式    float  6个        位置
//     static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20, 0x00, 0x00,
//                             // 			速度                 	力矩
//                             0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc,
//                             //			kp                	    kd
//                             0xcc, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                             //			最大力矩                占位（fdcan）
//                             0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x50, 0x50
//                            };

//     *(int32_t *)&cmd[14] = *(int32_t *)&torque;
//     // memcpy(&cmd[14], &torque, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
// }


// /**
//  * @brief 力矩控制 int32
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  * @param torque 力矩（单位见文档）
//  */
// void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque)
// {
//     //                     位置模式    		 int32  6个    	    位置
//     static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x08, 0x06, 0x20, 0x00, 0x00,
//                             //          速度                    力矩
//                             0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                             //          kp  					kd
//                             0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                             //          最大力矩   		 	    占位（fdcan）
//                             0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
//                            };

//     *(int32_t *)&cmd[14] = *(int32_t *)&torque;
//     // memcpy(&cmd[14], &torque, sizeof(int32_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
// }


// /**
//  * @brief 力矩控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  * @param torque 力矩（单位见文档）
//  */
// void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque)
// {
//     //                     		位置模式     int16   6个        位置
//     static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80,
//                             //速度      力矩        kp          kd          最大力矩    占位（fdcan）
//                             0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
//                            };

//     *(int16_t *)&cmd[10] = *(int16_t *)&torque;
//     // memcpy(&cmd[10], &torque, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置-速度-前馈力矩(最大力矩)控制，float型
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
//  * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
//  * @param torque 最大力矩（单位见文档）
//  */
// void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque)
// {
//     //                           位置模式     int32       位置                    速度                    			  力矩                    停止位置                占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0xcd, 0xcc, 0xcc, 0x3d, 0x0e, 0x25, 0x00, 0x00, 0x80, 0x3f, 0x9a, 0x99, 0x00, 0x00, 0x50};

//     // *(int32_t *)&cmd[9] = *(int32_t *)&vel;
//     // *(int32_t *)&cmd[15] = *(int32_t *)&torque;
//     // *(int32_t *)&cmd[19] = *(int32_t *)&pos;

//     memcpy(&cmd[9], &vel, sizeof(float));
//     memcpy(&cmd[15], &torque, sizeof(float));
//     memcpy(&cmd[19], &pos, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int32型
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
//  * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
//  * @param torque 最大力矩（单位见文档）
//  */
// void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque)
// {
//     //                           位置模式     int32       位置                    速度                    			  力矩                    停止位置                占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0a, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

//     // *(int32_t *)&cmd[9] =  vel;
//     // *(int32_t *)&cmd[13] = torque;
//     // *(int32_t *)&cmd[17] = pos;

//     memcpy(&cmd[9], &vel, sizeof(int32_t));
//     memcpy(&cmd[15], &torque, sizeof(int32_t));
//     memcpy(&cmd[19], &pos, sizeof(int32_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int16型
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param torque 最大力矩（单位见文档）
//  */
// void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque)
// {
//     //                            位置模式   2个int16      位置        速度		 2个int16	                力矩		  
//     static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x06, 0x25, 0x00, 0x00, 0x50, 0x50};

//     // *(int16_t *)&cmd[7] =  vel;
//     // *(int16_t *)&cmd[11] = torque;
//     // *(int16_t *)&cmd[13] = pos;

//     memcpy(&cmd[7], &vel, sizeof(int16_t));
//     memcpy(&cmd[11], &torque, sizeof(int16_t));
//     memcpy(&cmd[13], &pos, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置控制 float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
//  */
// void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos)
// {
//     //                           位置模式   1个float      位置                    占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[5], &pos, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置控制 int32
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
//  */
// void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos)
// {
//     //                           位置模式   1个int32      位置                    占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[5], &pos, sizeof(int32_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  */
// void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos)
// {
//     //                          位置模式     1个int16     位置
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x00};

//     memcpy(&cmd[5], &pos, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度控制 float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度：单位 1 转/秒，如 vel = 0.1 -> 0.1 转/秒
//  */
// void set_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel)
// {
//     //							  位置模式     2个float 	位置					速度					占位（fdcan）
//     static uint8_t cmd[16] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[9], &vel, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度控制 int32
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
//  */
// void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel)
// {
//     //							  位置模式     2个int32 	位置					速度					占位（fdcan）
//     static uint8_t cmd[16] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[9], &vel, sizeof(int32_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  */
// void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel)
// {
//     //							位置模式     2个int16 	  位置		  速度		  占位（fdcan）
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[7], &vel, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置、速度、前馈力矩、Kp、Kd控制 float 类型 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
//  * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
//  * @param tqe 前馈力矩：（单位见文档）
//  * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
//  * @param kd Mkd = kp * 1 (Mkd 表示电机内部 kd)
//  */
// void set_pos_vel_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd)
// {
//     static uint8_t cmd[] = {
//     0x01, 0x00, 0x0A,
//     0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x0e, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x50, 0x50, 0x50, 0x50, 0x50
//     };

//     memcpy(&cmd[5], &pos, sizeof(float));
//     memcpy(&cmd[9], &vel, sizeof(float));
//     memcpy(&cmd[13], &tqe, sizeof(float));
//     memcpy(&cmd[19], &kp, sizeof(float));
//     memcpy(&cmd[23], &kd, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机位置、速度、前馈力矩、Kp、Kd控制 int32 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
//  * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
//  * @param tqe 前馈力矩（单位见文档）
//  * @param kp Mkp = kp * 0.001 (Mkp 表示电机内部 kp)
//  * @param kd Mkd = kp * 0.001 (Mkd 表示电机内部 kd)
//  */
// void set_pos_vel_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd)
// {
//     static uint8_t cmd[] = {
//     0x01, 0x00, 0x0A,
//     0x0B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x0A, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x50, 0x50, 0x50, 0x50, 0x50
//     };
    
//     memcpy(&cmd[5], &pos, sizeof(float));
//     memcpy(&cmd[9], &vel, sizeof(float));
//     memcpy(&cmd[13], &tqe, sizeof(float));
//     memcpy(&cmd[19], &kp, sizeof(float));
//     memcpy(&cmd[23], &kd, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, 48);
// }


// /**
//  * @brief 电机位置、速度、前馈力矩、Kp、Kd控制 int16 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param tqe 前馈力矩（单位见文档）
//  * @param kp Mkp = kp * 0.1 (Mkp 表示电机内部 kp)
//  * @param kd Mkd = kp * 0.1 (Mkd 表示电机内部 kd)
//  */
// void set_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
// {
//     static uint8_t cmd[] = {
//     0x01, 0x00, 0x0A,
//     0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x06, 0x2b, 0x00, 0x00, 0x00, 0x00,
//     0x50, 0x50, 0x50
//     };

//     memcpy(&cmd[5], &pos, sizeof(int16_t));
//     memcpy(&cmd[7], &vel, sizeof(int16_t));
//     memcpy(&cmd[9], &tqe, sizeof(int16_t));
//     memcpy(&cmd[13], &kp, sizeof(int16_t));
//     memcpy(&cmd[15], &kd, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度、速度限幅控制（如果 vel > vel_max，则用 vel_max） int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param vel_max 速度限幅：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  */
// void set_vel_velmax_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t vel_max)
// {
//     //							位置模式				  位置        速度		  			  速度限制
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x28, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[11], &vel_max, sizeof(int16_t));
//     memcpy(&cmd[7], &vel, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 位置、速度、加速度限制（梯形控制） float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
//  * @param vel_max 速度限制，单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
//  * @param acc 加速度，单位：1 转/秒^2
//  */
// void set_pos_velmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc)
// {
//     // 							位置模式				  位置		  						  速度限制
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0E, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

//     memcpy(&cmd[5], &pos, sizeof(float));
//     memcpy(&cmd[11], &vel_max, sizeof(float));
//     memcpy(&cmd[15], &acc, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 位置、速度、加速度限制（梯形控制） int32
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
//  * @param vel_max 速度限制：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
//  * @param acc 加速度：单位 0.00001 转/秒^2，如 acc = 50000 表示 0.5 转/秒^2
//  */
// void set_pos_velmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc)
// {
//     // 							位置模式				  位置		  						  速度限制
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0A, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

//     memcpy(&cmd[5], &pos, sizeof(int32_t));
//     memcpy(&cmd[11], &vel_max, sizeof(int32_t));
//     memcpy(&cmd[15], &acc, sizeof(int32_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 位置、速度、加速度限制（梯形控制） int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
//  * @param vel_max 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param acc 加速度：单位 0.00025 转/秒^2，如 acc = 400 表示 0.1 转/秒^2
//  */
// void set_pos_velmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc)
// {
//     // 							位置模式				  位置		  速度限制
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x80, 0x06, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//     memcpy(&cmd[5], &pos, sizeof(int16_t));
//     memcpy(&cmd[9], &vel_max, sizeof(int16_t));
//     memcpy(&cmd[11], &acc, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度、加速度控制 float
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度，单位： 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
//  * @param acc 加速度，单位：1 转/秒^2
//  */
// void set_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc)
// {
//     //							位置模式				  位置					  速度								  加速度
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x0D, 0x29, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[9], &vel, sizeof(float));
//     memcpy(&cmd[15], &acc, sizeof(float));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度、加速度控制 int32
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
//  * @param acc 加速度：单位 0.001 转/秒^2，如 vel = 500 表示 0.5 转/秒^2
//  */
// void set_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc)
// {
//     //							位置模式				  位置					  速度								  加速度
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x09, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50, 0x50};

//     memcpy(&cmd[9], &vel, sizeof(int32_t));
//     memcpy(&cmd[15], &acc, sizeof(int32_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机速度、加速度控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param acc 加速度：单位 0.01 转/秒^2，如 vel = 40 表示 0.4 转/秒^2
//  */
// void set_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc)
// {
//     //							位置模式				  位置		  速度					  加速度
//     static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x29, 0x00, 0x00, 0x50, 0x50, 0x50};

//     memcpy(&cmd[7], &vel, sizeof(int16_t));
//     memcpy(&cmd[11], &acc, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 使用电压模式将电机固定（电机不会动，用于减少电机停止的声音，但是电流会增大）
//  * @param fdcanHandle &hfdcanx
//  * @param id 电机ID
//  * @param vol d相电压
//  */
// void set_vfoc_lock_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vol)
// {
//     static uint8_t cmd[] = {0x01, 0x00, 0x12, 0x05, 0x19, 0x00, 0x00};

//     memcpy(&cmd[5], &vol, sizeof(int16_t));

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样)
//  *          1. 周期返回电机位置、速度、力矩数据。
//  *          2. 返回数据格式和使用  0x17，0x01  指令获取的格式一样
//  *          3. 周期单位为 ms。
//  *          4. 最小周期为 1ms，最大周期 32767ms。
//  *          5. 如需停止周期返回数据，将周期给 0 即可，或者给电机断电。
//  * @param id 电机ID
//  * @param t 返回周期（单位：ms）
//  */
// void timed_return_motor_status_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t_ms)
// {
//     static uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

//     *(int16_t *)&tdata[3] = t_ms;

//     fdcan_send(fdcanHandle, 0x8000 | id, tdata, sizeof(tdata));
// }


// /**
//  * @brief 电机一拖多 位置控制 int16 
//  * @param fdcanHandle &hfdcanx
//  * @param pos1 电机1的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param pos2 电机2的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param pos3 电机3的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param pos4 电机4的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  */
// void set_many_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t pos2, int16_t pos3, int16_t pos4)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &pos1, sizeof(int16_t));
//     memcpy(&cmd[2], &pos2, sizeof(int16_t));
//     memcpy(&cmd[4], &pos3, sizeof(int16_t));
// 	memcpy(&cmd[6], &pos4, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_POSITION, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 速度控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param pos1 电机1的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param pos2 电机2的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param pos3 电机3的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param pos4 电机4的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  */
// void set_many_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t vel1, int16_t vel2, int16_t vel3, int16_t vel4)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &vel1, sizeof(int16_t));
//     memcpy(&cmd[2], &vel2, sizeof(int16_t));
//     memcpy(&cmd[4], &vel3, sizeof(int16_t));
// 	memcpy(&cmd[6], &vel4, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_VELOCITY, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 力矩控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param tqe1 电机1的力矩（单位见文档）
//  * @param tqe2 电机2的力矩（单位见文档）
//  * @param tqe3 电机3的力矩（单位见文档）
//  * @param tqe4 电机4的力矩（单位见文档）
//  */
// void set_many_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t tqe1, int16_t tqe2, int16_t tqe3, int16_t tqe4)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &tqe1, sizeof(int16_t));
//     memcpy(&cmd[2], &tqe2, sizeof(int16_t));
//     memcpy(&cmd[4], &tqe3, sizeof(int16_t));
// 	memcpy(&cmd[6], &tqe4, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_TORQUE, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 DQ电压控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param volt1 电机1的q相电压，单位：0.1V
//  * @param volt2 电机2的q相电压，单位：0.1V
//  * @param volt3 电机3的q相电压，单位：0.1V
//  * @param volt4 电机4的q相电压，单位：0.1V
//  */
// void set_many_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t volt1, int16_t volt2, int16_t volt3, int16_t volt4)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &volt1, sizeof(int16_t));
//     memcpy(&cmd[2], &volt2, sizeof(int16_t));
//     memcpy(&cmd[4], &volt3, sizeof(int16_t));
// 	memcpy(&cmd[6], &volt4, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_VOLTAGE, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 DQ电流控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param current1 电机1的q相电流，单位：0.1A
//  * @param current2 电机2的q相电流，单位：0.1A
//  * @param current3 电机3的q相电流，单位：0.1A
//  * @param current4 电机4的q相电流，单位：0.1A
//  */
// void set_many_current_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &current1, sizeof(int16_t));
//     memcpy(&cmd[2], &current2, sizeof(int16_t));
//     memcpy(&cmd[4], &current3, sizeof(int16_t));
// 	memcpy(&cmd[6], &current4, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_CURRENT, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 位置、速度、力矩控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param pos1 电机1的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel1 电机1的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param tqe1 电机1的力矩（单位见文档）
//  * @param pos2 电机2的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel2 电机2的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param tqe2 电机2的力矩（单位见文档）
//  */
// void set_many_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t tqe1, int16_t pos2, int16_t vel2, int16_t tqe2)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &pos1, sizeof(int16_t));
//     memcpy(&cmd[2], &vel1, sizeof(int16_t));
//     memcpy(&cmd[4], &tqe1, sizeof(int16_t));
    
//     memcpy(&cmd[6], &pos2, sizeof(int16_t));
//     memcpy(&cmd[8], &vel2, sizeof(int16_t));
//     memcpy(&cmd[10], &tqe2, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_POS_VEL_TQE, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 位置、速度、力矩、PD控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param pos1 电机1的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel1 电机1的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param tqe1 电机1的力矩（单位见文档）
//  * @param rkp1 电机1的 Kp 比例
//  * @param rkd1 电机1的 Kd 比例
//  * @param pos2 电机2的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel2 电机2的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param tqe2 电机2的力矩（单位见文档）
//  * @param rkp2 电机2的 Kp 比例
//  * @param rkd2 电机2的 Kd 比例
//  */
// void set_many_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t tqe1, int16_t rkp1, int16_t rkd1, int16_t pos2, int16_t vel2, int16_t tqe2, int16_t rkp2, int16_t rkd2)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &pos1, sizeof(int16_t));
//     memcpy(&cmd[2], &vel1, sizeof(int16_t));
//     memcpy(&cmd[4], &tqe1, sizeof(int16_t));
//     memcpy(&cmd[6], &rkp1, sizeof(int16_t));
//     memcpy(&cmd[8], &rkd1, sizeof(int16_t));
//     memcpy(&cmd[10], &pos2, sizeof(int16_t));
//     memcpy(&cmd[12], &vel2, sizeof(int16_t));
//     memcpy(&cmd[14], &tqe2, sizeof(int16_t));
//     memcpy(&cmd[16], &rkp2, sizeof(int16_t));
//     memcpy(&cmd[18], &rkd2, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_POS_VEL_TQE_KP_KD, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机一拖多 位置、速度、PD控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param pos1 电机1的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel1 电机1的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param rkp1 电机1的 Kp 比例
//  * @param rkd1 电机1的 Kd 比例
//  * @param pos2 电机2的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel2 电机2的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param rkp2 电机2的 Kp 比例
//  * @param rkd1 电机2的 Kd 比例
//  */
// void set_many_pos_vel_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t rkp1, int16_t rkd1, int16_t pos2, int16_t vel2, int16_t rkp2, int16_t rkd2)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &pos1, sizeof(int16_t));
//     memcpy(&cmd[2], &vel1, sizeof(int16_t));
//     memcpy(&cmd[4], &rkp1, sizeof(int16_t));
//     memcpy(&cmd[6], &rkd1, sizeof(int16_t));
    
//     memcpy(&cmd[8], &pos2, sizeof(int16_t));
//     memcpy(&cmd[10], &vel2, sizeof(int16_t));
//     memcpy(&cmd[12], &rkp2, sizeof(int16_t));
//     memcpy(&cmd[14], &rkd2, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_POS_VEL_KP_KD, cmd, sizeof(cmd));
// }



// /**
//  * @brief 电机一拖多 位置、速度、加速度控制 int16
//  * @param fdcanHandle &hfdcanx
//  * @param pos1 电机1的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel1 电机1的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param acc1 电机1的加速度，单位 0.01 转/秒^2，如 vel = 40 表示 0.4 转/秒^2
//  * @param pos2 电机2的位置，单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
//  * @param vel2 电机2的速度，单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
//  * @param acc2 电机2的加速度，单位 0.01 转/秒^2，如 vel = 40 表示 0.4 转/秒^2
//  */
// void set_many_pos_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t acc1, int16_t pos2, int16_t vel2, int16_t acc2)
// {
//     static uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01};

//     memcpy(&cmd[0], &pos1, sizeof(int16_t));
//     memcpy(&cmd[2], &vel1, sizeof(int16_t));
//     memcpy(&cmd[4], &acc1, sizeof(int16_t));
    
//     memcpy(&cmd[6], &pos2, sizeof(int16_t));
//     memcpy(&cmd[8], &vel2, sizeof(int16_t));
//     memcpy(&cmd[10], &acc2, sizeof(int16_t));

//     fdcan_send(fdcanHandle, MODE_POS_VEL_ACC, cmd, sizeof(cmd));
// }


// /**
//  * @brief 重设电机零位
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     static uint8_t cmd[] = {0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73, 0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20, 0x30, 0x2e, 0x30, 0x0a};

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));

//     HAL_Delay(1000);

//     set_conf_write(fdcanHandle, 1);
// }


// /**
//  * @brief 保存电机设置
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     static uint8_t cmd[] = {0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50};

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机停止，注意：需让电机停止后再重置零位，否则无效
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     static uint8_t cmd[] = {0x01, 0x00, 0x00};

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 电机刹车
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     static uint8_t cmd[] = {0x01, 0x00, 0x0f};

//     fdcan_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
// }


// /**
//  * @brief 获取电机状态 float，状态、位置、速度、转矩
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     const uint8_t cmd[] = {0x1C, 0x04, 0x00};

//     fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
// }


// /**
//  * @brief 获取电机状态 int32，状态、位置、速度、转矩
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     const uint8_t cmd[] = {0x18, 0x04, 0x00};

//     fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
// }


// /**
//  * @brief 获取电机状态 int16，状态、位置、速度、转矩
//  * @param fdcanHandle &hfdcanx
//  * @param id id 电机ID
//  */
// void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
// {
//     const uint8_t cmd[] = {0x14, 0x04, 0x00};

//     fdcan_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
// }


// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
//     uint8_t len = 0;
//     if(hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)
//     {
//         HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header1, fdcan1_rdata);
//         if (fdcan_rx_header1.DataLength != 0)
//         {

//             len = get_fdcan_data_size(fdcan_rx_header1.DataLength);
//             motor_state.motor.id = fdcan_rx_header1.Identifier;  // 获取电机 id
// #if READ_MOTOR_FLAG == 1 || READ_MOTOR_FLAG == 2 || READ_MOTOR_FLAG == 3
//             memcpy(&motor_state.data[0], &fdcan1_rdata[3], len - 3);  // 获取电机状态数据
// #endif
//             motor_read_flag = 1;
//         }
//     }
// }



