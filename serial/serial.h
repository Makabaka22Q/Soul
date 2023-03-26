//
// Created by redamancy on 2023/3/9.
//
#ifndef ATS_23_VISION_SERIAL_H
#define ATS_23_VISION_SERIAL_H

/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>
#include "fmt/color.h"

#include <iostream>
#include <vector>

//#include "../general/general.h"

#include "CRC_Check.h"
#include "../debug.h"

using namespace std;

#define TRUE 1
#define FALSE 0

//模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //自瞄
#define CmdID3 0x03; //小符
#define CmdID4 0x04; //大符

// C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

//默认串口最大编号
const int MAX_ITER = 3;

//字节数为4的结构体
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

//用于保存目标相关角度和距离信息及瞄准情况
//发送结构体(视觉->电控)
typedef struct
{
    float2uchar pitch_angle; //偏航角
    float2uchar yaw_angle;   //俯仰角
    float2uchar dis;  //目标距离
    float2uchar oppo_speed;   //对方速度
    float2uchar oppo_acc;   //对方加速度
    int isSpinning;   //目标是否处于陀螺状态
    //int isFindTarget; //当识别的图片范围内有目标
    int shoot_flag;  //击打标志
} VisionData;

class serial
{
public:
    atomic_bool need_init;
    int fd;      //串口号
    int speed;  //波特率
    int baud;
    int mode;
    int color;
    int databits, stopbits;
    int parity;  //奇偶校验位
    unsigned char rdata[255];                 // raw_data
    float quat[4]; //四元数
    float acc[3]; //加速度
    float gyro[3]; //角速度
    float bullet_speed;
    serial(const string name, const int BUAD);
    ~serial();
    int set_opt(int fd, int speed, char parity, int databits, int stopbits);
    bool InitPort(int speed = 115200, char parity = 'N', int databits = 8, int stopbits = 1);
    bool get_Mode();
    void TransformData(const VisionData &data); //主要方案
    void send();
    void closePort();
private:
    unsigned char Tdata[30];                  // transfrom data

    string serial_name;

    float exchange_data(unsigned char *data); //将4个uchar合并成一个float
    bool getQuat(unsigned char *data);
    bool getGyro(unsigned char *data);
    bool getAcc(unsigned char *data);
    bool getSpeed(unsigned char *data);

};

#endif //ATS_23_VISION_SERIAL_H
