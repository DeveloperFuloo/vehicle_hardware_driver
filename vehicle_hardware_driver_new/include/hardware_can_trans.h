/***************************************
Copyright Guangdong Jaten Robot & Automation Co.,Ltd. 2020-2029. All rights reserved
FileName: can_trans.h
Description: can transmit
AuthorName: luyi
AuthorID: 1952
E-mail: luyi@jtrobots.com
Date: 2021.9.3
Modify: luyi,2021.9.3,created
*****************************************/
#ifndef __CAN_TRANS_H_
#define __CAN_TRANS_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <stdint-gcc.h>
#include <ros/ros.h>
#include <fcntl.h>

using namespace std;
typedef struct can_frame CanFrame_t;

typedef struct{
    int socket_handle;
    struct ifreq ifr;
    CanFrame_t frame_send;
    CanFrame_t frame_read;
}Can_CFG_t;

typedef enum{
    CAN_0 = 0,
    CAN_1 = 1
}Can_Type_e;

class CanRx_TX{
public:
CanRx_TX();
~CanRx_TX();
bool Read_One_Msg(CanFrame_t *data2read);
bool Write_One_Msg(CanFrame_t *data2send);
bool Read_muti_Msg(vector<CanFrame_t>* data2read);

private:
Can_CFG_t can0;
};

#endif  //__CAN_TRANS_H_