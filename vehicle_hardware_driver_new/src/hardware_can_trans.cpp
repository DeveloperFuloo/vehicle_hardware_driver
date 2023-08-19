/***************************************
Copyright Guangdong Jaten Robot & Automation Co.,Ltd. 2020-2029. All rights reserved
FileName: hardware_can_trans.cpp
Description: can transmit
AuthorName: luyi
AuthorID: 1952
E-mail: luyi@jtrobots.com
Date: 2021.9.3
Modify: luyi,2021.9.3,created
*****************************************/
#include "hardware_can_trans.h"

/*******************************************************************************
 * @brief  	构造函数
 * @param 	NONE
 * @return	NONE
*******************************************************************************/
CanRx_TX::CanRx_TX(){
    struct sockaddr_can addr;


    //todo:设置can的波特率

    memset(&can0.frame_read, 0, sizeof(can0.frame_read));
    memset(&can0.frame_send, 0, sizeof(can0.frame_send));
    strcpy(can0.ifr.ifr_name, "can0" );
    can0.socket_handle = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    ioctl(can0.socket_handle, SIOCGIFINDEX, &can0.ifr); //指定 can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = can0.ifr.ifr_ifindex;
    bind(can0.socket_handle, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定
    fcntl(can0.socket_handle, F_SETFL, FNDELAY);//将read设置成非阻塞
    can0.frame_send.can_id = 0x01;
    can0.frame_send. can_dlc = 8;
}

/*******************************************************************************
 * @brief  	析构函数
 * @param 	NONE
 * @return	NONE
*******************************************************************************/
CanRx_TX::~CanRx_TX(){

}

/*******************************************************************************
 * @brief   读取一帧消息
 * @param   data2read：保存读取的一帧数据
 * @param   type:can端口，can0或can1
 * @return	读取成功则为true.
*******************************************************************************/
bool CanRx_TX::Read_One_Msg(CanFrame_t *data2read){
    int nbytes;

    nbytes = read(can0.socket_handle, data2read, sizeof(CanFrame_t));
    
    return (nbytes == sizeof(CanFrame_t))? true:false;
}

/*******************************************************************************
 * @brief   读取多帧消息
 * @param   data2read：保存读取的数据
 * @param   type:can端口，can0或can1
 * @param   num:读取帧数
 * @return	读取成功则为true.
*******************************************************************************/
bool CanRx_TX::Read_muti_Msg(vector<CanFrame_t>* data2read){
    CanFrame_t data_tmp;
    data2read->resize(0);
 
    for(int i=0; i< 10; i++)
    {
        if(read(can0.socket_handle, &data_tmp, sizeof(CanFrame_t)) == sizeof(CanFrame_t)){
            data2read->push_back(data_tmp);
        }else{
            break;
        }
    }
  
    return (data2read->size() != 0)? true:false;
}

/*******************************************************************************
 * @brief  	写一帧消息
 * @param 	data2send：要写的数据
 * @return	写成功则返回true
*******************************************************************************/
bool CanRx_TX::Write_One_Msg(CanFrame_t *data2send){
    int nbytes;
    
    nbytes = write(can0.socket_handle, data2send, sizeof(CanFrame_t));
    
    return (nbytes == sizeof(CanFrame_t))? true:false;
}
