/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"

#pragma pack(1)
typedef struct
{
    int16_t vol; // 电压
    int16_t referee_power; // 电流
    int16_t chassis_power; // 功率

    float cap_real_vol; // 实际电压
    float referee_real_chassis_power; // 实际电流
    float chassis_real_power; // 实际功率

    uint16_t time;
} SuperCap_Msg_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    SuperCap_Msg_s cap_msg; // 超级电容信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 * 
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 * 
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

#endif // !SUPER_CAP_Hd
