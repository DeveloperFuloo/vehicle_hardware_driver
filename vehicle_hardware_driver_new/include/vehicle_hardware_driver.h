/***************************************
Copyright Guangdong Jaten Robot & Automation Co.,Ltd. 2020-2029. All rights reserved
FileName: jt_vehicle_hardware_driver.h
Description: chassis by wire driver
AuthorName: luyi
AuthorID: 1952
E-mail: luyi@jtrobots.com
Date: 2021.5.14
Modify: luyi,2021.5.14,created
*****************************************/
#ifndef CHASSIS_DRIVER_H
#define CHASSIS_DRIVER_H
#include "stdint.h"
#include "stdio.h"
#include <ros/ros.h>
#include <ros/time.h>
#include "hardware_can_trans.h"
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <map>
#include <vehicle_hardware_driver/DoorLightDebug.h>

namespace HardwareNS{
// ros::Time begin = ros::Time::now();
// double secs =ros::Time::now().toSec();
using DLDebug = vehicle_hardware_driver::DoorLightDebug;

typedef enum{
  E_DOOR1 = 0,
  E_DOOR2,
  E_DOOR3,
  E_DOOR4,
  E_LIGHT_LEFT,
  E_LIGHT_RIGHT,
  E_LIGHT_WIDTH,
  E_LIGHT_LATTICE,
  E_LIGHT_EMBIENCE,
  E_CABIN_LAMP,
  E_LIGHT_ERROR,
  E_LIGHT_BRAKE,
  E_LIGHT_ALARM1,
  E_LIGHT_ALARM2,
  E_LIGHT_ALARM3,
  E_LIGHT_ALARM4
}EquipmentType;

/*********************CAN0消息ID************************/
typedef enum{
  // //车门、车灯控制
  DOOR_CONTROL            = 0x251,//车门控制
  DOOR_TRIP               = 0x261,
  DOOR_STATUS             = 0x262,
  ULTRASONIC_0            = 0x710,  //超声波消息0
  ULTRASONIC_1            = 0x711, //超声波消息1
  CANIO_WRITE_RELAY       = 0x109,   // CAN_IO 地址为0x09
  CANIO_READ_RELAY        = 0x209,
  CANIO_READ_RELAY_INPUT  = 0x309,
  CANIO_SET_RELAY_PARAM   = 0x409   // 设置参数，一般提前设置好，

}CAN0_msg_id;

//16 位控制,给can模块发送的值，控制can模块的IO
typedef enum{
  WITH_LAMP           = 0x0003,   //示廓灯
  AMBIENCE_LAMP       = 0x000C,   //氛围灯 
  BRAK_ERRO_LAMP      = 0x0014,
  LATTICE_LAMP        = 0x060,    //点阵灯
  LEFT_TURN_LAMP      = 0x0200,
  RIGHT_TURN_LAMP     = 0x0400,
  DOOR_ARARM1_LAMP    = 0x0080,
  DOOR_ARARM2_LAMP    = 0x0100,
  DOOR_ARARM3_LAMP    = 0x0800,
  DOOR_ARARM4_LAMP    = 0x1000,
  CABIN_LAMP          = 0x8000
}LightTriggle;

typedef enum{
  OFF = 0,
  ON = 1,
  TURN = 2
}LamPState;

typedef struct {
  uint8_t Trip1Fdk;  //1号门行程（开度），取值范围0-100，对应门的关与开
  uint8_t Trip2Fdk;
  uint8_t Trip3Fdk;
  uint8_t Trip4Fdk;
  uint8_t door_ctr_status;  //门单独控制
  uint8_t door_error_status;  
}DoorStatus_t;


typedef struct {
  uint8_t cmd[8];
  bool update;
}Cmd_t;

typedef struct {
  uint8_t state;  //0:检测到障碍物 1:未检测到障碍物，2：探头未插入，3：探头不需要工作
  uint16_t dist;  //障碍物距离,当检测到障碍物时有效
}Ultra_t;

typedef struct {
  Ultra_t ults[12];
  bool update;
  uint8_t precision; // 精度，0表示探头的精度为1cm  1表示探头的精度为2cm,2表示探头的精度1cm和2cm都有
}UltraArray_t;      



class LampBase{
public:
  LampBase(uint16_t tri, uint16_t twin=0){
    triggle = tri;
    twinkle = twin;
    m_desired_state = 0;
    current_state = 0;
    update = false;
    count = 0;
  }
  
  void set_desired_state(LamPState desired_state){
    m_desired_state = desired_state;
    if(m_desired_state == 2){
      count = 0;
      current_state = 0;
    }
  }

  uint8_t get_desired_state(){
    return m_desired_state;
  }

  uint16_t get_state_triggle(){
    if(m_desired_state == 0){
      return 0;
    }else if(m_desired_state == 1){
      return triggle;
    }else if(m_desired_state == 2){
      if(twinkle < count++)
      {
        update = true;
        count = 0;
        if(current_state){       //之前亮，现在要灭
          current_state = false;
          // if(triggle == BRAK_ERRO_LAMP){   //默认氛围灯常亮
          //   return AMBIENCE_LAMP;
          // }
          return 0;
        }else{                   //之前灭，现在要亮
          current_state = true;
          return triggle;
        }
      }
    }
  }
 
  void set_update(bool val){
    update = val;
  }

  bool get_update(){
    return update;
  }

private:
  uint16_t triggle;   //打开时的码
  uint8_t m_desired_state;  //0:希望关闭，1：希望打开，2：希望闪烁
  bool current_state;  //当前是打开则为1,当前是关闭则为0
  uint16_t twinkle; //闪烁频率（非严格）
  uint16_t count;   //计数器
  bool update;
};

class LampManager{
public:
  enum Fre{
    HZ_1 = 50,
    HZ_2 = 25,
    HZ_5 = 10
  };
  LampManager(){
    lamps[WITH_LAMP]        = make_shared<LampBase>(WITH_LAMP,      HZ_2);
    lamps[AMBIENCE_LAMP]    = make_shared<LampBase>(AMBIENCE_LAMP,  HZ_2);
    lamps[BRAK_ERRO_LAMP]   = make_shared<LampBase>(BRAK_ERRO_LAMP, HZ_2);
    // lamps[BRAK_LAMP] = make_shared<LampBase>(BRAK_LAMP);
    lamps[LATTICE_LAMP]     = make_shared<LampBase>(LATTICE_LAMP,    HZ_2);
    // lamps[BACK_LATTICE_ LAMP] = make_shared<LampBase>(BACK_LATTICE_LAMP);
    lamps[LEFT_TURN_LAMP]   = make_shared<LampBase>(LEFT_TURN_LAMP,  HZ_2);
    lamps[RIGHT_TURN_LAMP]  = make_shared<LampBase>(RIGHT_TURN_LAMP, HZ_2);

    lamps[DOOR_ARARM1_LAMP] = make_shared<LampBase>(DOOR_ARARM1_LAMP,HZ_2);
    lamps[DOOR_ARARM2_LAMP] = make_shared<LampBase>(DOOR_ARARM2_LAMP,HZ_2);
    lamps[DOOR_ARARM3_LAMP] = make_shared<LampBase>(DOOR_ARARM3_LAMP,HZ_2);
    lamps[DOOR_ARARM4_LAMP] = make_shared<LampBase>(DOOR_ARARM4_LAMP,HZ_2);
    lamps[CABIN_LAMP]       = make_shared<LampBase>(CABIN_LAMP,      HZ_2);
  }

  void LampScan(){  
    //获取期望的值
    lamp_triggle = 0;
    lamp_triggle |=lamps[WITH_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[AMBIENCE_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[BRAK_ERRO_LAMP]->get_state_triggle();
    // lamp_triggle |=lamps[BRAK_LAMP]->get_state_triggle();
    // printf("triggle1:%x\n",lamp_triggle);
    lamp_triggle |=lamps[LATTICE_LAMP]->get_state_triggle();
    // lamp_triggle |=lamps[BACK_LATTICE_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[LEFT_TURN_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[RIGHT_TURN_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[DOOR_ARARM1_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[DOOR_ARARM2_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[DOOR_ARARM3_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[DOOR_ARARM4_LAMP]->get_state_triggle();
    lamp_triggle |=lamps[CABIN_LAMP]->get_state_triggle();
    // printf("triggle2:%x\n",lamps[LEFT_TURN_LAMP]->get_state_triggle());

    //获取更新状态
    if(lamps[WITH_LAMP]->get_update()){
      update |=lamps[WITH_LAMP]->get_update();
      lamps[WITH_LAMP]->set_update(false);
    }

    if(lamps[AMBIENCE_LAMP]->get_update()){
      update |=lamps[AMBIENCE_LAMP]->get_update();
      lamps[AMBIENCE_LAMP]->set_update(false);
    }

    if(lamps[BRAK_ERRO_LAMP]->get_update()){
      update |=lamps[BRAK_ERRO_LAMP]->get_update();
      lamps[BRAK_ERRO_LAMP]->set_update(false);
    }

    // if(lamps[BRAK_LAMP]->get_update()){
    //   update |=lamps[BRAK_LAMP]->get_update();
    //   lamps[BRAK_LAMP]->set_update(false);
    // }

    if(lamps[LATTICE_LAMP]->get_update()){
      update |=lamps[LATTICE_LAMP]->get_update();
      lamps[LATTICE_LAMP]->set_update(false);
    }

    if(lamps[LEFT_TURN_LAMP]->get_update()){
      update |=lamps[LEFT_TURN_LAMP]->get_update();
      lamps[LEFT_TURN_LAMP]->set_update(false);
    }


    if(lamps[RIGHT_TURN_LAMP]->get_update()){
      update |=lamps[RIGHT_TURN_LAMP]->get_update();
      lamps[RIGHT_TURN_LAMP]->set_update(false);
    }

    if(lamps[DOOR_ARARM1_LAMP]->get_update()){
      update |=lamps[DOOR_ARARM1_LAMP]->get_update();
      lamps[DOOR_ARARM1_LAMP]->set_update(false);
    }

    if(lamps[DOOR_ARARM2_LAMP]->get_update()){
      update |=lamps[DOOR_ARARM2_LAMP]->get_update();
      lamps[DOOR_ARARM2_LAMP]->set_update(false);
    }   
    
    if(lamps[DOOR_ARARM3_LAMP]->get_update()){
      update |=lamps[DOOR_ARARM3_LAMP]->get_update();
      lamps[DOOR_ARARM3_LAMP]->set_update(false);
    }

    if(lamps[DOOR_ARARM4_LAMP]->get_update()){
      update |=lamps[DOOR_ARARM4_LAMP]->get_update();
      lamps[DOOR_ARARM4_LAMP]->set_update(false);
    }

    if(lamps[CABIN_LAMP]->get_update()){
      update |=lamps[CABIN_LAMP]->get_update();
      lamps[CABIN_LAMP]->set_update(false);
    }   
    
  }

  void set_update(bool val){
    update = val;
  }

  bool get_update(){
    return update;
  }

  uint16_t get_triggle(){
    return lamp_triggle;
  }

  map<LightTriggle,shared_ptr<LampBase>>  lamps;
private:
  bool update;  //所有灯中，是否有更新
  uint16_t lamp_triggle;
};

class VehicleHardwareDriver{
public:
  VehicleHardwareDriver();
  ~VehicleHardwareDriver();
  void receive_decoder(void);
  void start();
  void DoorLightStatusPub();
  bool IsDoor_ok();
  bool IsLight_OK();
  void SendData2Vehicle(uint8_t* data, uint32_t id);
  void DisplayUltrs();
  void DoorCmdManager(const DLDebug& cmd);
  void LightCmdManager(const DLDebug& cmd);

  void LightCmdCallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
  void DoorCmdCallback(const std_msgs::UInt8MultiArrayConstPtr& msg);

  void DoorDebugCallback(const vehicle_hardware_driver::DoorLightDebugConstPtr& msg);
  void LightDebugCallback(const vehicle_hardware_driver::DoorLightDebugConstPtr& msg);
private:
  ros::Publisher door_status_pub;
  ros::Publisher lightAndDoorStatus_pub;
  ros::Publisher ultra_data_pub;

  ros::Subscriber light_cmd_sub;
  ros::Subscriber door_cmd_sub;
  ros::Subscriber door_debug_sub;
  ros::Subscriber light_debug_sub;

  CanRx_TX can_msg_class;
  DoorStatus_t door_status;  //灯和门的状态反馈
  // uint8_t ctr_cmd[8];
  Cmd_t door_cmd;
  Cmd_t light_cmd;

  map<CAN0_msg_id,int> msg_type;  //有效的消息ID集合
  std_msgs::UInt8MultiArray ultrasonic;
  UltraArray_t ultra;   //超声波数据
  DLDebug cur_command;
  LampManager lamp_manager;
};

}//end of namespace

#endif  //end CHASSIS_DRIVER_H



