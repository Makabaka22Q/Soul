//
// Created by redamancy on 2023/3/11.
//
#ifndef ATS_23_VISION_DAHENGCAMERA_H
#define ATS_23_VISION_DAHENGCAMERA_H

#include "./DxImageProc.h"
#include "./GxIAPI.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fmt/format.h>
#include <fmt/color.h>

using namespace std;
using namespace cv;

//本类现只应对使用单个相机的情况
class DaHengCamera
{
private:
    //枚举操作成功与失败信息
    GX_STATUS status = GX_STATUS_SUCCESS;
    //相机设备
    GX_DEV_HANDLE hDevice = NULL;
    //定 义 GXDQBuf 的 传 入 参 数,包含图片内存，大小等信息
    PGX_FRAME_BUFFER pFrameBuffer;

    bool                set_contrast;               ///< Contrast Set
    bool                set_color;
    bool                set_saturation;

    int                 timestamp_offset = 0;
    int64_t             m_i64ColorCorrection;       ///< Color correction param
    int64_t             lastImgTimestamp;           ///< timestamp of last img
    void*               pGammaLut;                  ///< Gamma look up table
    int                 nLutLength;                 ///< Gamma look up table length
    void*               pContrastLut;             ///< Contrast look up table
    int                 m_nContrastLutLength;       ///< Contrast look up table length
    double              dGammaParam;
    VxInt32             contrast_factor;                    ///< Contrast Set factor
    VxInt32             saturation_factor;
    int64_t             nContrastParam;
    int64_t             nColorCorrectionParam;
    VxInt16             nSaturation;

public:
    //构造函数，初始化库
    DaHengCamera();
    //打开设备
    int StartDevice(int serial_number);
    //使设备开始采集
    bool SetStreamOn();
    //设置分辨率，支持1:1(最大1280*1024),1:2,2:1,2:2(最小640*512),默认1:1
    bool SetResolution(int width_scale = 1, int height_scale = 1);
    //手动设置曝光值,单位us,正常大小应在2000至8000
    bool SetExposureTime(int ExposureTime);
    //设置曝光增益
    bool SetGAIN(int value, int ExpGain);
    //设置自动白平衡,0表示关闭，1表示开启
    bool Set_BALANCE_AUTO(int value);
    //手动设置白平衡,value表示平衡通道，value_number表示具体值,0、1、2对应B、G、R，value_number范围为10到80,10表示正常
    bool Set_BALANCE(int value, float value_number);
    //Color_Correct
    bool Color_Correct(bool value);
    // Set Gamma
    bool Set_Gamma(bool set_status,double dGammaParam);
    // Set Contrast
    bool Set_Contrast(bool set_status,int dContrastParam);
    // Set_Saturation
    bool Set_Saturation(bool set_status,int dSaturationParam);
    //采集一次图像,更新时间戳
    bool UpdateTimestampOffset(std::chrono::_V2::steady_clock::time_point time_start);
    //采集图像
    bool GetMat(Mat &Src);
    //读取相机时间戳
    int Get_TIMESTAMP();
    //析构函数释放资源
    ~DaHengCamera();
};

#endif //ATS_23_VISION_DAHENGCAMERA_H
