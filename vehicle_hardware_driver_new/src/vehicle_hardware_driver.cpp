/***************************************
Copyright Guangdong Jaten Robot & Automation Co.,Ltd. 2020-2029. All rights reserved
FileName:vehicle_hardware_driver.cpp
Description: chassis by wire driver
AuthorName: luyi
AuthorID: 1952
E-mail: luyi@jtrobots.com
Date: 2021.5.14
Modify: luyi,2021.5.14,created
*****************************************/
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
// #include <pthread.h>

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "vehicle_hardware_driver.h"
#include "std_msgs/Float32.h"

namespace HardwareNS{

#define _IS_DEBUG_  1
/*******************************************************************************
 * @brief  	构造函数
 * @param 	NONE
 * @return	NONE
*******************************************************************************/
VehicleHardwareDriver::VehicleHardwareDriver(){
	ros::NodeHandle nh_private("~");
	msg_type.clear();
	msg_type[DOOR_CONTROL] = 0;
	msg_type[DOOR_TRIP] = 0;
	msg_type[DOOR_STATUS] = 0;
	msg_type[ULTRASONIC_0] = 0;
	msg_type[ULTRASONIC_1] = 0;
	msg_type[CANIO_WRITE_RELAY] = 0;
	msg_type[CANIO_READ_RELAY] = 0;
	msg_type[CANIO_READ_RELAY_INPUT] = 0;
	msg_type[CANIO_SET_RELAY_PARAM] = 0;

	memset(&door_cmd, 0, 8);
	door_cmd.cmd[0] = 0xaa;
	door_cmd.cmd[1] = 0xaa;
	memset(&ultra, 0, sizeof(ultra));

  	door_status_pub = nh_private.advertise<std_msgs::Bool>("/vehicle_chassis/door_status",10);
	lightAndDoorStatus_pub = nh_private.advertise<std_msgs::UInt8MultiArray>("/vehicle_chassis/light_and_door_status",10);
	ultra_data_pub = nh_private.advertise<std_msgs::UInt8MultiArray>("/ultrasonic_data",10);

	light_cmd_sub = nh_private.subscribe("/behavior_manager/light_cmd2driver", 10, &VehicleHardwareDriver::LightCmdCallback, this);
	door_cmd_sub = nh_private.subscribe("/behavior_manager/door_cmd2driver", 10, &VehicleHardwareDriver::DoorCmdCallback, this);

	if(_IS_DEBUG_){
		door_debug_sub = nh_private.subscribe("/debug/door_cmd2driver", 10, &VehicleHardwareDriver::DoorDebugCallback, this);
		light_debug_sub = nh_private.subscribe("/debug/light_cmd2driver", 10, &VehicleHardwareDriver::LightDebugCallback, this);
	}

}

/*******************************************************************************
 * @brief  	析构函数
 * @param 	NONE
 * @return	NONE
*******************************************************************************/
VehicleHardwareDriver::~VehicleHardwareDriver(){}

void VehicleHardwareDriver::DoorLightStatusPub(){
	std_msgs::UInt8MultiArray data2send;
	//顺序不能调换
	data2send.data.push_back(door_status.Trip1Fdk);  //byte 0
	data2send.data.push_back(door_status.Trip2Fdk);  //byte 1
	data2send.data.push_back(door_status.Trip3Fdk);  //byte 2
	data2send.data.push_back(door_status.Trip4Fdk);  //byte 3

	
	data2send.data.push_back(door_status.door_ctr_status & 0x03);  //byte 4 ，门1状态
	data2send.data.push_back((door_status.door_ctr_status>>2) & 0x03);  //byte 5 ，门2状态
	data2send.data.push_back((door_status.door_ctr_status>>4) & 0x03);  //byte 6 ，门3状态
	data2send.data.push_back((door_status.door_ctr_status>>6) & 0x03);  //byte 7 ，门4状态

	data2send.data.push_back(door_status.door_error_status & 0x01);  //byte 8 ,门1故障状态
	data2send.data.push_back((door_status.door_error_status>>1) & 0x01);  //byte 9 ,门2故障状态
	data2send.data.push_back((door_status.door_error_status>>2) & 0x01);  //byte 10 ,门3故障状态
	data2send.data.push_back((door_status.door_error_status>>3) & 0x01);  //byte 11 ,门4故障状态

	data2send.data.push_back(lamp_manager.lamps[LEFT_TURN_LAMP]->get_desired_state());  //byte 12
	data2send.data.push_back(lamp_manager.lamps[RIGHT_TURN_LAMP]->get_desired_state());  //byte 13
	data2send.data.push_back(lamp_manager.lamps[WITH_LAMP]->get_desired_state());  //byte 14
	data2send.data.push_back(lamp_manager.lamps[LATTICE_LAMP]->get_desired_state());  //byte 15
	data2send.data.push_back(lamp_manager.lamps[AMBIENCE_LAMP]->get_desired_state());  //byte 16
	data2send.data.push_back(lamp_manager.lamps[CABIN_LAMP]->get_desired_state());  //byte 17

	if(lamp_manager.lamps[BRAK_ERRO_LAMP]->get_desired_state() == 0){
		data2send.data.push_back(0);  //byte 18  error
		data2send.data.push_back(0);  //byte 19  brake
	}else if(lamp_manager.lamps[BRAK_ERRO_LAMP]->get_desired_state() == 1){
		data2send.data.push_back(0);  //byte 18
		data2send.data.push_back(1);  //byte 19
	}else{
		data2send.data.push_back(1);  //byte 18
		data2send.data.push_back(0);  //byte 19
	}

	data2send.data.push_back(lamp_manager.lamps[DOOR_ARARM1_LAMP]->get_desired_state());  //byte 20
	data2send.data.push_back(lamp_manager.lamps[DOOR_ARARM2_LAMP]->get_desired_state());  //byte 21
	data2send.data.push_back(lamp_manager.lamps[DOOR_ARARM3_LAMP]->get_desired_state());  //byte 22
	data2send.data.push_back(lamp_manager.lamps[DOOR_ARARM4_LAMP]->get_desired_state());  //byte 23

	lightAndDoorStatus_pub.publish(data2send);	
}

bool VehicleHardwareDriver::IsDoor_ok(){
	// if(door_light_status.door_error_status & 0x0f){  //门有故障，目前不区分哪个门
	// 	return false;
	// }

	return true;
}

void VehicleHardwareDriver::LightCmdManager(const DLDebug& cmd){
	lamp_manager.set_update(true);
	uint8_t equit = cmd.vehicle;
	uint8_t e_value = cmd.cmd;
	switch(equit){
		//左转向灯
		case E_LIGHT_LEFT:{
			if(e_value){
				lamp_manager.lamps[LEFT_TURN_LAMP]->set_desired_state(TURN);
				lamp_manager.lamps[RIGHT_TURN_LAMP]->set_desired_state(OFF);
			}else{
				lamp_manager.lamps[LEFT_TURN_LAMP]->set_desired_state(OFF);
			}
			
			break;
		}

		//右转向灯
		case E_LIGHT_RIGHT:{
			if(e_value){
				lamp_manager.lamps[LEFT_TURN_LAMP]->set_desired_state(OFF);
				lamp_manager.lamps[RIGHT_TURN_LAMP]->set_desired_state(TURN);
			}else{
				lamp_manager.lamps[RIGHT_TURN_LAMP]->set_desired_state(OFF);
			}

			break;			
		}

		//示廓灯
		case E_LIGHT_WIDTH:{
			if(e_value){
				lamp_manager.lamps[WITH_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[WITH_LAMP]->set_desired_state(OFF);
			}
			
			break;	
		}

		//点阵灯
		case E_LIGHT_LATTICE:{
			if(e_value){
				lamp_manager.lamps[LATTICE_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[LATTICE_LAMP]->set_desired_state(OFF);
			}

			break;		
		}

		//氛围灯
		case E_LIGHT_EMBIENCE:{
			if(e_value){
				lamp_manager.lamps[AMBIENCE_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[AMBIENCE_LAMP]->set_desired_state(OFF);
			}

			break;		
		}

		//货舱灯
		case E_CABIN_LAMP:{
			if(e_value){
				lamp_manager.lamps[CABIN_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[CABIN_LAMP]->set_desired_state(OFF);
			}
			
			break;	
		}

		//故障灯
		case E_LIGHT_ERROR:{
			if(e_value){
				// lamp_manager.lamps[AMBIENCE_LAMP]->set_desired_state(ON);
				lamp_manager.lamps[BRAK_ERRO_LAMP]->set_desired_state(TURN);
			}else{

				lamp_manager.lamps[BRAK_ERRO_LAMP]->set_desired_state(OFF);
			}

			break;		
		}

		//刹车灯
		case E_LIGHT_BRAKE:{
			if(e_value){
				// lamp_manager.lamps[AMBIENCE_LAMP]->set_desired_state(ON);
				lamp_manager.lamps[BRAK_ERRO_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[BRAK_ERRO_LAMP]->set_desired_state(OFF);
			}

			break;		
		}

		//门1故障灯
		case E_LIGHT_ALARM1:{
			if(e_value){
				lamp_manager.lamps[DOOR_ARARM1_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[DOOR_ARARM1_LAMP]->set_desired_state(OFF);
			}

			break;		
		}

		//门2故障灯
		case E_LIGHT_ALARM2:
		{
			if(e_value){
				lamp_manager.lamps[DOOR_ARARM2_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[DOOR_ARARM2_LAMP]->set_desired_state(OFF);
			}

			break;		
		}

		//门3故障灯
		case E_LIGHT_ALARM3:{
			if(e_value){
				lamp_manager.lamps[DOOR_ARARM3_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[DOOR_ARARM3_LAMP]->set_desired_state(OFF);
			}

			break;		
		}
		//门4故障灯
		case E_LIGHT_ALARM4:{
			if(e_value){
				lamp_manager.lamps[DOOR_ARARM4_LAMP]->set_desired_state(ON);
			}else{
				lamp_manager.lamps[DOOR_ARARM4_LAMP]->set_desired_state(OFF);
			}

			break;		
		}
	}

}

void VehicleHardwareDriver::door_process(){

}

void VehicleHardwareDriver::DoorCmdManager(const DLDebug& cmd){
	uint8_t equit = cmd.vehicle;
	uint8_t e_value = cmd.cmd;
	door_cmd.update = true;
	lamp_manager.set_update(true);

	switch(equit){
		//door 1
		case E_DOOR1:{
			if(e_value == 0){
				door_cmd.cmd[0] = door_cmd.cmd[0] & 0xfc | 0x01;
				
				//lamp_manager.lamps[DOOR_ARARM1_LAMP]->set_desired_state(OFF);
			}else if(e_value == 1){
				//door_cmd.cmd[0] = door_cmd.cmd[0] & 0xfc | 0x02;

				door_cmd.cmd[0] = door_cmd.cmd[0] & 0xfc | 0x02;-+
				-	
				//lamp_manager.lamps[DOOR_ARARM1_LAMP]->set_desired_state(TURN);
			}else if(e_value == 2){
				door_cmd.cmd[0] &= 0xfc;
				//lamp_manager.lamps[DOOR_ARARM1_LAMP]->set_desired_state(TURN);
			}
			
			break;					
		}

		//door 2
		case E_DOOR2:{
			if(e_value == 0){
				door_cmd.cmd[0] = door_cmd.cmd[0] & 0xf3 | 0x04;

				//lamp_manager.lamps[DOOR_ARARM2_LAMP]->set_desired_state(OFF);
			}else if(e_value == 1){

				door_cmd.cmd[0] = door_cmd.cmd[0] & 0xf3 | 0x08;
				//lamp_manager.lamps[DOOR_ARARM2_LAMP]->set_desired_state(TURN);
			}else if(e_value == 2){
				door_cmd.cmd[0] &= 0xf3;
				//lamp_manager.lamps[DOOR_ARARM2_LAMP]->set_desired_state(TURN);
			}
			
			break;					
		}

		//door 3
		case E_DOOR3:{
			if(e_value == 0){
				door_cmd.cmd[0] = door_cmd.cmd[0] & 0xcf | 0x10;

				//lamp_manager.lamps[DOOR_ARARM3_LAMP]->set_desired_state(OFF);
			}else if(e_value == 1){

				door_cmd.cmd[0] = door_cmd.cmd[0] & 0xcf | 0x20;
				//lamp_manager.lamps[DOOR_ARARM3_LAMP]->set_desired_state(TURN);
			}else if(e_value == 2){
				door_cmd.cmd[0] &= 0xcf;
				//lamp_manager.lamps[DOOR_ARARM3_LAMP]->set_desired_state(TURN);
			}
			
			break;					
		}

		//door 4
		case E_DOOR4:{
			if(e_value == 0){

				door_cmd.cmd[0] = door_cmd.cmd[0] & 0x3f | 0x40;
				//lamp_manager.lamps[DOOR_ARARM4_LAMP]->set_desired_state(OFF);
			}else if(e_value == 1){

				door_cmd.cmd[0] = door_cmd.cmd[0] & 0x3f | 0x80;
				//lamp_manager.lamps[DOOR_ARARM4_LAMP]->set_desired_state(TURN);
			}else if(e_value == 2){
				door_cmd.cmd[0] &= 0x3f;
				//lamp_manager.lamps[DOOR_ARARM4_LAMP]->set_desired_state(TURN);
			}
			
			break;					
		}
	}
}

void VehicleHardwareDriver::LightCmdCallback(const std_msgs::UInt8MultiArrayConstPtr& msg){
	if(msg->data.size() != 2){   //两个参数
		return; 
	}

	cur_command.vehicle = msg->data[0];
	cur_command.cmd = msg->data[1];

	LightCmdManager(cur_command);
}

void VehicleHardwareDriver::DoorCmdCallback(const std_msgs::UInt8MultiArrayConstPtr& msg){
	if(msg->data.size() != 2){   //两个参数
		return; 
	}


	cur_command.vehicle = msg->data[0];
	cur_command.cmd = msg->data[1];
	DoorCmdManager(cur_command);
	printf("tmp_cmd.vehicle: %x ,tmp_cmd.cmd :%x",cur_command.vehicle, cur_command.cmd);
}

void VehicleHardwareDriver::DoorDebugCallback(const vehicle_hardware_driver::DoorLightDebugConstPtr& msg){
	DoorCmdManager(*msg);
}

void VehicleHardwareDriver::LightDebugCallback(const vehicle_hardware_driver::DoorLightDebugConstPtr& msg){
	LightCmdManager(*msg);
}


void VehicleHardwareDriver::SendData2Vehicle(uint8_t* data, uint32_t id){
	CanFrame_t send_data;

	//send_data
	send_data.can_id = id;
	send_data.can_dlc = 8;
	memcpy(&send_data.data, data, 8);
	
	can_msg_class.Write_One_Msg(&send_data);
}

/*******************************************************************************
 * @brief  	read CAN msg and decode them
 * @param th:this pointer
 * @return	NONE
*******************************************************************************/
void VehicleHardwareDriver::receive_decoder(void){
	vector<CanFrame_t> rec_0;//can0接收缓存。
	bool Can0_msg_update = false;  //
	bool Can1_msg_update = false; 

	uint16_t tmp_uint16;
	uint8_t  tmp_uint8;
	uint32_t tmp_uint32;
	int16_t tmp_int16;
 
	Can0_msg_update = can_msg_class.Read_muti_Msg(&rec_0);

	//CAN0消息
	if(Can0_msg_update){
		Can0_msg_update = false;

		for(int i=0; i<rec_0.size(); i++){
			CAN0_msg_id  rec_id = static_cast<CAN0_msg_id>(rec_0[i].can_id);
			if(!msg_type.count(rec_id)){  //不存在该消息
				continue;
			}
			switch(rec_0[i].can_id){
				case DOOR_TRIP:{
					door_status.Trip1Fdk = rec_0[i].data[0];
					door_status.Trip2Fdk = rec_0[i].data[1];
					door_status.Trip3Fdk = rec_0[i].data[2];
					door_status.Trip4Fdk = rec_0[i].data[3];
					break;
				}
				case DOOR_STATUS:{
					door_status.door_ctr_status = rec_0[i].data[0];
					door_status.door_error_status = rec_0[i].data[1];
					printf ("door_status : %x\n",rec_0[i].data[0]);
					break;
				}
				case ULTRASONIC_0:{
					ultra.precision = (rec_0[i].data[1]>>6) & 0x3;  //低两位
					ultra.update = true;
					//1号超声波
					if(rec_0[i].data[4] == 0xfc){ //未检测到障碍物
						ultra.ults[0].state = 1;
						ultra.ults[0].dist = 251;
					}else if(rec_0[i].data[4] == 0xfd){ //探头未插入
						ultra.ults[0].state = 2;
						ultra.ults[0].dist = 251;
					}else if(rec_0[i].data[4] == 0xfe){//探头不需要工作
						ultra.ults[0].state = 3;
						ultra.ults[0].dist = 251;
					}else{
						ultra.ults[0].state = 0;
						ultra.ults[0].dist = rec_0[i].data[4];
					}

					//2号超声波
					if(rec_0[i].data[5] == 0xfc){ //未检测到障碍物
						ultra.ults[1].state = 1;
						ultra.ults[1].dist = 251;
					}else if(rec_0[i].data[5] == 0xfd){ //探头未插入
						ultra.ults[1].state = 2;
						ultra.ults[1].dist = 251;
					}else if(rec_0[i].data[5] == 0xfe){//探头不需要工作
						ultra.ults[1].state = 3;
						ultra.ults[1].dist = 251;
					}else{
						ultra.ults[1].state = 0;
						ultra.ults[1].dist = rec_0[i].data[5];
					}

					//3号超声波
					if(rec_0[i].data[6] == 0xfc){ //未检测到障碍物
						ultra.ults[2].state = 1;
						ultra.ults[2].dist = 251;
					}else if(rec_0[i].data[6] == 0xfd){ //探头未插入
						ultra.ults[2].state = 2;
						ultra.ults[2].dist = 251;
					}else if(rec_0[i].data[6] == 0xfe){//探头不需要工作
						ultra.ults[2].state = 3;
						ultra.ults[2].dist = 251;
					}else{
						ultra.ults[2].state = 0;
						ultra.ults[2].dist = rec_0[i].data[6];
					}

					//4号超声波
					if(rec_0[i].data[7] == 0xfc){ //未检测到障碍物
						ultra.ults[3].state = 1;
						ultra.ults[3].dist = 251;
					}else if(rec_0[i].data[7] == 0xfd){ //探头未插入
						ultra.ults[3].state = 2;
						ultra.ults[3].dist = 251;
					}else if(rec_0[i].data[7] == 0xfe){//探头不需要工作
						ultra.ults[3].state = 3;
						ultra.ults[3].dist = 251;
					}else{
						ultra.ults[3].state = 0;
						ultra.ults[3].dist = rec_0[i].data[7];
					}
					break;
				}
				case ULTRASONIC_1:{
					ultra.update = true;
					//5号超声波
					if(rec_0[i].data[0] == 0xfc){ //未检测到障碍物
						ultra.ults[4].state = 1;
						ultra.ults[4].dist = 251;
					}else if(rec_0[i].data[0] == 0xfd){ //探头未插入
						ultra.ults[4].state = 2;
						ultra.ults[4].dist = 251;
					}else if(rec_0[i].data[0] == 0xfe){//探头不需要工作
						ultra.ults[4].state = 3;
						ultra.ults[4].dist = 251;
					}else{
						ultra.ults[4].state = 0;
						ultra.ults[4].dist = rec_0[i].data[0];
					}

					//6号超声波
					if(rec_0[i].data[1] == 0xfc){ //未检测到障碍物
						ultra.ults[5].state = 1;
						ultra.ults[5].dist = 251;
					}else if(rec_0[i].data[1] == 0xfd){ //探头未插入
						ultra.ults[5].state = 2;
						ultra.ults[5].dist = 251;
					}else if(rec_0[i].data[1] == 0xfe){//探头不需要工作
						ultra.ults[5].state = 3;
					    ultra.ults[5].dist = 251;
					}else{
						ultra.ults[5].state = 0;
						ultra.ults[5].dist = rec_0[i].data[1];
					}

					//7号超声波
					if(rec_0[i].data[2] == 0xfc){ //未检测到障碍物
						ultra.ults[6].state = 1;
						ultra.ults[6].dist = 251;
					}else if(rec_0[i].data[2] == 0xfd){ //探头未插入
						ultra.ults[6].state = 2;
						ultra.ults[6].dist = 251;
					}else if(rec_0[i].data[2] == 0xfe){//探头不需要工作
						ultra.ults[6].state = 3;
						ultra.ults[6].dist = 251;
					}else{
						ultra.ults[6].state = 0;
						ultra.ults[6].dist = rec_0[i].data[2];
					}

					//8号超声波
					if(rec_0[i].data[3] == 0xfc){ //未检测到障碍物
						ultra.ults[7].state = 1;
						ultra.ults[7].dist = 251;
					}else if(rec_0[i].data[3] == 0xfd){ //探头未插入
						ultra.ults[7].state = 2;
						ultra.ults[7].dist = 251;
					}else if(rec_0[i].data[3] == 0xfe){//探头不需要工作
						ultra.ults[7].state = 3;
						ultra.ults[7].dist = 251;
					}else{
						ultra.ults[7].state = 0;
						ultra.ults[7].dist = rec_0[i].data[3];
					}

					//9号超声波
					if(rec_0[i].data[4] == 0xfc){ //未检测到障碍物
						ultra.ults[8].state = 1;
						ultra.ults[8].dist = 251;
					}else if(rec_0[i].data[4] == 0xfd){ //探头未插入
						ultra.ults[8].state = 2;
						ultra.ults[8].dist = 251;
					}else if(rec_0[i].data[4] == 0xfe){//探头不需要工作
						ultra.ults[8].state = 3;
						ultra.ults[8].dist = 251;
					}else{
						ultra.ults[8].state = 0;
						ultra.ults[8].dist = rec_0[i].data[4];
					}

					//10号超声波
					if(rec_0[i].data[5] == 0xfc){ //未检测到障碍物
						ultra.ults[9].state = 1;
						ultra.ults[9].dist = 251;
					}else if(rec_0[i].data[5] == 0xfd){ //探头未插入
						ultra.ults[9].state = 2;
						ultra.ults[9].dist = 251;
					}else if(rec_0[i].data[5] == 0xfe){//探头不需要工作
						ultra.ults[9].state = 3;
						ultra.ults[9].dist = 251;
					}else{
						ultra.ults[9].state = 0;
						ultra.ults[9].dist = rec_0[i].data[5];
					}

					//11号超声波
					if(rec_0[i].data[6] == 0xfc){ //未检测到障碍物
						ultra.ults[10].state = 1;
						ultra.ults[10].dist = 251;
					}else if(rec_0[i].data[6] == 0xfd){ //探头未插入
						ultra.ults[10].state = 2;
						ultra.ults[10].dist = 251;
					}else if(rec_0[i].data[6] == 0xfe){//探头不需要工作
						ultra.ults[10].state = 3;
						ultra.ults[10].dist = 251;
					}else{
						ultra.ults[10].state = 0;
						ultra.ults[10].dist = rec_0[i].data[6];
					}

					//12号超声波
					if(rec_0[i].data[7] == 0xfc){ //未检测到障碍物
						ultra.ults[11].state = 1;
						ultra.ults[11].dist = 251;
					}else if(rec_0[i].data[7] == 0xfd){ //探头未插入
						ultra.ults[11].state = 2;
						ultra.ults[11].dist = 251;
					}else if(rec_0[i].data[7] == 0xfe){//探头不需要工作
						ultra.ults[11].state = 3;
						ultra.ults[11].dist = 251;
					}else{
						ultra.ults[11].state = 0;
						ultra.ults[11].dist = rec_0[i].data[7];
					}
					//顺序,从左前方开始,顺时针
			
					std_msgs::UInt8MultiArray ultra_send;
					ultra_send.data.clear();
					ultra_send.data.push_back(ultra.ults[4].dist);
					ultra_send.data.push_back(ultra.ults[3].dist);
					ultra_send.data.push_back(ultra.ults[2].dist);
					ultra_send.data.push_back(ultra.ults[1].dist);
					ultra_send.data.push_back(ultra.ults[5].dist);
					ultra_send.data.push_back(ultra.ults[11].dist);
					ultra_send.data.push_back(ultra.ults[10].dist);
					ultra_send.data.push_back(ultra.ults[9].dist);

					ultra_data_pub.publish(ultra_send);
					break;
				}
				default:{}
			} //end switch
		} //end for
		// ROS_INFO("------------------------------*");

	}
}

/*******************************************************************************
 * @brief : main thread of the chassis driver
 * @param : NONE
 * @return :NONE
*******************************************************************************/
void VehicleHardwareDriver::start(){
	ros::Rate r(50); //发布频率
	//发布门心跳信息
	std_msgs::Bool heart_beat_pub;

	/*for debug*/
	// lamp_manager.set_update(true);
	// lamp_manager.lamps[RIGHT_TURN_LAMP]->set_desired_state(TURN);
	// lamp_manager.lamps[LEFT_TURN_LAMP]->set_desired_state(OFF);
	// lamp_manager.lamps[AMBIENCE_LAMP]->set_desired_state(OFF);
	// lamp_manager.lamps[BRAK_ERRO_LAMP]->set_desired_state(ON);
	// lamp_manager.lamps[LATTICE_LAMP]->set_desired_state(TURN);
	// uint8_t test_s[8] = {};
	// test_s[0] = 0x14;
	// test_s[1] = 0x00;
	// SendData2Vehicle(test_s, CANIO_WRITE_RELAY);
	/*for debug end*/

	while(ros::ok()){
		ros::spinOnce();
		//接收CAN数据
		receive_decoder();

		//发布节点状态，用于系统诊断
		heart_beat_pub.data = IsDoor_ok()? true:false;
		door_status_pub.publish(heart_beat_pub);

		//灯和门的状态发布
		DoorLightStatusPub();
		
		// door
		if(door_cmd.update){   //门控制指令有更新，发送到底盘
		// uint8_t data = 0x02;
		// SendData2Vehicle(&data, DOOR_CONTROL);
			if(cur_command.cmd == 1 && door_status.door_ctr_status ==)
			SendData2Vehicle(door_cmd.cmd, DOOR_CONTROL);
		door_cmd.update = false;
		printf("send door cmd: %x\n",door_cmd.cmd[0]);
		}
		
		//lamp
		lamp_manager.LampScan();

		if(lamp_manager.get_update()){
			uint8_t data_send[8];
			// array<uint8_t,8> data_send(0);
			// vector<uint8_t> data_send(8,0);
			// data_send.resize(8);
			memset(data_send, 0, sizeof(data_send));
			uint16_t tri = lamp_manager.get_triggle();
			data_send[0] = tri & 0xff;
			data_send[1] = (tri>>8) & 0xff;
			SendData2Vehicle(data_send, CANIO_WRITE_RELAY);
			// printf("flag11,data_send:%x, id: %x\n",tri,CANIO_WRITE_RELAY);
			lamp_manager.set_update(false);
		}

		//ulrasonic
		if(ultra.update){
			// DisplayUltrs();
			ultra.update = false;
		}
		
		r.sleep();
	}

	//goto ext;
}

void VehicleHardwareDriver::DisplayUltrs(){
	printf("**************ultrs********************\n");
	printf("********precision %d\n",ultra.precision);
	for(int i=0; i<12; i++){
		if(i==0 || i==6 || i==7 || i==8){
			continue;
		}
		// printf("********index %d,state: %d\n",i+1,ultra.ults[i].state);
		// if(ultra.ults[i].state == 0){
		// 	printf("********i=%d,dist: %d\n",i,ultra.ults[i].dist);
		// }

		printf("********i=%d,dist: %d\n",i+1,ultra.ults[i].dist);
	}
	// int i = 1;
	// static int k = 0;
	// printf("********magic:%d,index %d,state: %d,dist: %d\n",k++,i+1, ultra.ults[i].state, ultra.ults[i].dist);
	// if(k>9){
	// 	k = 0;
	// }
	//2->FR
	//3->FMR
	//4->FML
	//5->FL
	//10->RL
	//11->RML
	//12->RMR
	//6->RR
}

}//end of namespace
