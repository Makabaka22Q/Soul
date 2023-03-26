//
// Created by redamancy on 2023/3/9.
//
#include "thread.h"
#include "../general/general.h"
#include <fmt/core.h>

/**
 * @brief 生产者线程
 * @param factory 工厂类
 * 相机模块
**/
bool producer(Factory<TaskData> &factory, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start) {
#ifdef USING_DAHENG
    start_get_img:
    DaHengCamera DaHeng;
    DaHeng.StartDevice(1);
    // 设置分辨率
    DaHeng.SetResolution(1,1);
    //更新时间戳，设置时间戳偏移量
    DaHeng.UpdateTimestampOffset(time_start);
    // 开始采集帧
    DaHeng.SetStreamOn();
    // 设置曝光事件
    DaHeng.SetExposureTime(3500);
    // 设置1
    DaHeng.SetGAIN(3, 16);
    // 是否启用自动白平衡7
    // DaHeng.Set_BALANCE_AUTO(0);
    // manual白平衡 BGR->012
    DaHeng.Set_BALANCE(0,1.56);
    DaHeng.Set_BALANCE(1,1.0);
    DaHeng.Set_BALANCE(2,1.548);
    // // Gamma
    // DaHeng.Set_Gamma(1,1.0);
    // //Color
    // DaHeng.Color_Correct(1);
    // //Contrast
    // DaHeng.Set_Contrast(1,10);
    // //Saturation
    // DaHeng.Set_Saturation(0,0);
#endif //USING_DAHENG

#ifdef USING_VIDEO
    sleep(6);
    VideoCapture cap("/home/redamancy/桌面/ATS-23-Vision/video/armor_red");
#endif

    fmt::print(fmt::fg(fmt::color::green), "[CAMERA] Set param finished\n");

    while (1)
    {
        TaskData src;
        auto time_cap = std::chrono::steady_clock::now();
#ifdef USING_DAHENG
        auto DaHeng_smat=DaHeng.GetMat(src.img);
        if (!DaHeng_smat)
        {
            fmt::print(fmt::fg(fmt::color::red), "[CAMERA] GetMat false return\n");

            #ifdef SAVE_LOG_ALL
                LOG(ERROR) << "[CAMERA] GetMat false return";
            #endif //SAVE_LOG_ALL

            goto start_get_img;   //到15行位置，设置相机参数
        }
        src.timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
#endif
    }
}

/**
 * @brief 消费者线程
 * @param factory 工厂类   任务 传输
**/
bool consumer(Factory<TaskData> &task_factory,Factory<VisionData> &transmit_factory)
{


}

/**
 * @brief 数据发送线程
 *
 * @param serial SerialPort类
 * @param transmit_factory Factory类
 * @return true
 * @return false
 */
bool dataTransmitter(serial &serial,Factory<VisionData> &transmit_factory)
{
    while(1)
    {
        VisionData transmit;
        transmit_factory.consume(transmit);
        serial.TransformData(transmit);
        serial.send();
    }
}

//#ifdef USING_IMU_C_BOARD
/**
 * @brief 数据接收线程
 *
 *
 * @param serial
 * @param receive_factory
 * @param time_start
 * @return true
 * @return false
 */
bool dataReceiver(serial &serial,MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start)
{
    while(1)
    {

        while(!serial.get_Mode());
        auto time_cap = std::chrono::steady_clock::now();
        auto timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
        int mode = serial.mode;
        int color=serial.color;
        float bullet_speed = serial.bullet_speed;

        Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
        Eigen::Vector3d acc = {serial.acc[0],serial.acc[1],serial.acc[2]};
        Eigen::Vector3d gyro = {serial.gyro[0],serial.gyro[1],serial.gyro[2]};
        MCUData mcu_status = {mode, color, acc, gyro, quat, bullet_speed, timestamp};
        receive_factory.produce(mcu_status, timestamp);

    }
    return true;
}

//#endif


/**
 * @brief 串口监视线程
 *
 * @param serial
 * @return true
 * @return false
 */
 bool serialWatcher(serial &serial)
{
     int last=0;
     while(1)
     {
         sleep(1);

     }
}

