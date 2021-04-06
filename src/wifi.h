/**********************************Copyright (c)**********************************
**                       版权所有 (C), 2015-2020, 涂鸦科技
**
**                             http://www.tuya.com
**
*********************************************************************************/
/**
 * @file    wifi.h
 * @author  涂鸦综合协议开发组
 * @version v2.5.6
 * @date    2020.12.16
 * @brief   用户无需关心该文件实现内容
 */

/****************************** 免责声明 ！！！ *******************************
由于MCU类型和编译环境多种多样，所以此代码仅供参考，用户请自行把控最终代码质量，
涂鸦不对MCU功能结果负责。
******************************************************************************/


#ifndef __WIFI_H_
#define __WIFI_H_
//#include<HardwareSerial.h>
//#include "stm32f1xx.h"
#include "stdio.h"
#include "string.h"
#include "protocol.h"
#include "system.h"
#include "mcu_api.h"
#include<Arduino.h>

//*****************引脚宏定义**************************
#define WIFI_STATE_PIN	PC13	//WiFi状态信号灯，低电平亮灯
#define MUP_PIN			PA0		//电机上升限位，下拉电阻，高电平限位生效
#define MDOWN_PIN		PA1		//电机下降限位，下拉电阻，高电平限位生效
#define FI1_PIN			PA2		//出粮电机
#define BI1_PIN			PA3		//出粮电机
#define FI2_PIN			PB0		//升降电机
#define BI2_PIN			PB1		//升降电机
#define CHKWATER_PIN	PA4		//5v上拉电阻10k,水位传感器变阻器模拟输入。
#define WATER_PIN		PB10	//水泵驱动mos G极引脚，高电平有效
#define OKBTN3_PIN		PB9		//OK按钮，内部下拉,高电平有效
#define DOWNBTN2_PIN		PB8		//下按钮，内部下拉,高电平有效
#define UPBTN_PIN		PB5		//上按钮，内部下拉,高电平有效
#define SCL_LED			PB6		//i2c
#define SDA_LED			PB7		//i2c
//#define SWDCLK_PIN	PA14
//#define SWDIO_PIN	PA13
//#define USBDP PA12
//#define USBDM PA11
//#define RX PA10
//#define TX PA9
//**************************************************
extern void ty_write(unsigned char value);

//********************全局变量***********************


 //喂食状态 枚举型数据上报;枚举范围：standby,feeding,done
enum feed_state
{
	standby, feeding, done
};

extern bool display_flash_flag;//显示屏刷新标记。true刷新
extern bool up_button_flag;//上按钮标记
extern bool down_button_flag ;//下按钮标记
extern bool ok_button_flag;//ok按钮标记
//**************************************************

//****************可下发可上报DP***********************
extern  unsigned char* ty_meal_plan ;//喂食计划 最大长度：128
extern  bool ty_quick_feed;	//快速喂食 BOOL型数据上报;
extern int ty_manual_feed; //手动喂食 VALUE型数据上报;数值范围：1-12，间距：1，单位：null
//**************************************************


//****************只上报DP***********************
 //喂食状态 枚举型数据上报;枚举范围：standby,feeding,done
extern enum feed_state ty_feed_state;
extern int ty_battery_percentage; //电池电量 VALUE型数据上报;数值范围：0-100，间距：1，单位：%
extern bool ty_charge_state; //充电状态 BOOL型数据上报;
//ty_fault; //故障型数据上报;标签：pet_food_jam,pet_food_shortages,pet_food_run_out,desiccant_exhausted,battery_low
extern int ty_feed_report; //喂食结果 VALUE型数据上报;数值范围：0-12，间距：1，单位：null
extern int ty_surplus_grain; //当前粮桶余粮 VALUE型数据上报;数值范围：0-100，间距：1，单位：%
//**************************************************





//=============================================================================
//定义常量
//如果编译发生错误: #40: expected an identifier  DISABLE = 0, 类似这样的错误提示，可以包含头文件 #include "stm32f1xx.h" 来解决
//=============================================================================
#ifndef TRUE
#define      TRUE                1
#endif

#ifndef FALSE
#define         FALSE            0
#endif

#ifndef NULL
#define         NULL             ((void *) 0)
#endif

#ifndef SUCCESS
#define         SUCCESS          1
#endif

#ifndef ERROR
#define         ERROR            0
#endif

#ifndef INVALID
#define         INVALID          0xFF
#endif

#ifndef ENABLE
#define         ENABLE           1
#endif

#ifndef DISABLE
#define         DISABLE          0
#endif
//=============================================================================
//dp数据点类型
//=============================================================================
#define         DP_TYPE_RAW                     0x00        //RAW 类型
#define         DP_TYPE_BOOL                    0x01        //bool 类型
#define         DP_TYPE_VALUE                   0x02        //value 类型
#define         DP_TYPE_STRING                  0x03        //string 类型
#define         DP_TYPE_ENUM                    0x04        //enum 类型
#define         DP_TYPE_BITMAP                  0x05        //fault 类型

//=============================================================================
//WIFI工作状态
//=============================================================================
#define         SMART_CONFIG_STATE              0x00
#define         AP_STATE                        0x01
#define         WIFI_NOT_CONNECTED              0x02
#define         WIFI_CONNECTED                  0x03
#define         WIFI_CONN_CLOUD                 0x04
#define         WIFI_LOW_POWER                  0x05
#define         SMART_AND_AP_STATE              0x06
#define         WIFI_SATE_UNKNOW                0xff
//=============================================================================
//wifi配网的方式
//=============================================================================
#define         SMART_CONFIG                    0x0  
#define         AP_CONFIG                       0x1   

//=============================================================================
//wifi复位状态
//=============================================================================
#define         RESET_WIFI_ERROR                0
#define         RESET_WIFI_SUCCESS              1

//=============================================================================
//wifi配置复位状态
//=============================================================================
#define         SET_WIFICONFIG_ERROR            0
#define         SET_WIFICONFIG_SUCCESS          1

//=============================================================================
//MCU固件升级状态
//=============================================================================
#define         FIRM_STATE_UN_SUPPORT           0x00                            //不支持 MCU 升级
#define         FIRM_STATE_WIFI_UN_READY        0x01                            //模块未就绪
#define         FIRM_STATE_GET_ERROR            0x02                            //云端升级信息查询失败
#define         FIRM_STATE_NO                   0x03                            //无需升级（云端无更新版本）
#define         FIRM_STATE_START                0x04                            //需升级，等待模块发起升级操作

//=============================================================================
//WIFI和mcu的工作方式 
//=============================================================================
#define         UNION_WORK                      0x0                             //mcu模块与wifi配合处理
#define         WIFI_ALONE                      0x1                             //wifi模块自处理

//=============================================================================
//系统工作模式
//=============================================================================
#define         NORMAL_MODE                     0x00                            //正常工作状态
#define         FACTORY_MODE                    0x01                            //工厂模式	
#define         UPDATE_MODE                     0x02                            //升级模式	 

//=============================================================================
//配网方式选择
//=============================================================================
#define         CONFIG_MODE_DEFAULT             "0"                             //默认配网方式
#define         CONFIG_MODE_LOWPOWER            "1"                             //低功耗配网方式
#define         CONFIG_MODE_SPECIAL             "2"                             //特殊配网方式  




//=============================================================================
//下发命令
//=============================================================================
typedef struct {
  unsigned char dp_id;                              //dp序号
  unsigned char dp_type;                            //dp类型
} DOWNLOAD_CMD_S;

#endif
