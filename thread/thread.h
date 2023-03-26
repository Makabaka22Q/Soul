//
// Created by redamancy on 2023/3/9.
//
#include "../camera/DaHengCamera.h"
#include "../autoaim/autoaim.h"
#include "../buff/buff.h"
#include "../serial/serial.h"
#include "../debug.h"
#include "../general/general.h"

#include <iterator>
#include <thread>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

/**
 * 接收 结构体
 **/
struct MCUData
{
    int mode;
    int color; //敌方颜色
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Quaterniond quat;
    double bullet_speed;
    int timestamp;
};

/**
 * 工厂类
 * */
template <typename T>
class Factory
{
private:
    std::deque<T> buffer;  //队列
    int buffer_size;
    mutex lock;

public:
    /**
     * @brief 工厂类初始化
     * @param size 队列长度
    **/
    Factory(int size)
    {
        buffer_size = size;
    };
    bool produce(T &product);
    bool consume(T &product);
};

/**
 * 生产队列 push_back 入栈
 * */
template <typename T>
bool Factory<T>::produce(T &product)
{
    lock.lock();
    if (buffer.size() < buffer_size)
        buffer.push_back(product);   //在buffer中添加product
    else
    {
        buffer.pop_front();  //移除首元素，在buffer中添加
        buffer.push_back(product);
    }
    lock.unlock();

    return true;
}

/**
 * 消费  buffer非空时，将buffer首个元素给product
 * */
template <typename T>
bool Factory<T>::consume(T &product)
{
    while (1)
    {
        lock.lock();
        if (!buffer.empty())
            break;
        lock.unlock();
        usleep(1e3); //阻塞1e3微秒，直到有未被阻塞信号到达
    }
    product = buffer.front();
    buffer.pop_front();
    lock.unlock();

    return true;
}

/**
 * 消息过滤器
 * */
template <typename T>
class MessageFilter
{
private:
    struct Product
    {
        T message;
        int timestamp;
    };
    std::deque<Product> buffer;
    atomic_bool is_editing;  //编辑
    mutex lock;
    int buffer_size;
public:
    /**
     * @brief 工厂类初始化
     * @param size 队列长度
    **/
    MessageFilter(int size)
    {
        buffer_size = size;
        is_editing = false;
    };
    bool produce(T &message, int timestamp);
    bool consume(T &message, int timestamp);
};


template <typename T>
bool MessageFilter<T>::produce(T &message, int timestamp)
{
    lock.lock();
Product product = {message, timestamp};   //
    if (buffer.size() < buffer_size)
        buffer.push_back(product);
    else
    {
        buffer.pop_front();
        buffer.push_back(product);
    }
    lock.unlock();

    return true;
}

template <typename T>
bool MessageFilter<T>::consume(T &message, int timestamp) {
    //队列为空时阻塞消费者
    while (1) {
        lock.lock();
        if (!buffer.empty())
            break;
        lock.unlock();
        usleep(1e3);
    }
    auto it = std::lower_bound(buffer.begin(), buffer.end(), timestamp,
                               [](Product &prev, const int &timestamp) { return prev.timestamp < timestamp; });
    if (it == buffer.end()) {
        //时间戳时间差大于10ms则认为该帧不可用
        if (abs((buffer.back().timestamp - timestamp)) > 10) {
            buffer.pop_front();
            lock.unlock();
            return false;
        } else {
            message = (buffer.back()).message;
            buffer.pop_front();
            lock.unlock();
            return true;
        }
    } else {
        it--;
        message = (*it).message;
        buffer.erase(it);auto it = std::lower_bound(buffer.begin(), buffer.end(), timestamp, [](Product &prev, const int &timestamp)
                               { return prev.timestamp < timestamp; });
    if (it == buffer.end())
    {
        //时间戳时间差大于10ms则认为该帧不可用
        if (abs((buffer.back().timestamp - timestamp)) > 10)
        {
            buffer.pop_front();
            lock.unlock();
            return false;
        }
        else
        {
            message = (buffer.back()).message;
            buffer.pop_front();
            lock.unlock();
            return true;
        }
    }
    else
    {
        it--;
        message = (*it).message;
        buffer.erase(it);
    }
    lock.unlock();
    return true;
    }
    lock.unlock();
    return true;
}

bool producer(Factory<TaskData> &factory, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start);
bool consumer(Factory<TaskData> &task_factory, Factory<VisionData> &transmit_factory);
bool dataTransmitter(serial &serial, Factory<VisionData> &transmit_factory);

#ifdef USING_IMU_C_BOARD
bool dataReceiver(serial &serial, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start);
bool serialWatcher(serial &serial);
#endif
#ifndef USING_IMU
bool serialWatcher(serial &serial);
#endif
