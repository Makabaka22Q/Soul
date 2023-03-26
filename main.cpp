#include <iostream>
#include "./thread/thread.h"
#include "./serial/serial.h"

#include "debug.h"
#include "camera/DaHengCamera.h"
#include "general/general.h"

DaHengCamera DaHeng;
//默认串口名
const string serial_name= "ttyUSB0";
const int BAUD = 115200;

int main(int argc,char* argv[])
{
#ifdef SAVE_MAIN_LOG
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = false;  //除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = true;  //是否启用不同颜色显示

    // 下面的函数暂时没有必要开启
    // FLAGS_logbufsecs = 0;   //设置可以缓冲日志的最大秒数，0指实时输出
    // FLAGS_max_log_size = 10;  //日志文件大小(单位：MB)
    // FLAGS_stop_logging_if_full_disk = true; //磁盘满时是否记录到磁盘

    google::SetLogDestination(google::GLOG_INFO,"../log/info/");  //设置日志级别
    google::SetLogDestination(google::GLOG_WARNING,"../log/warning/");
    google::SetLogDestination(google::GLOG_ERROR,"../log/error/");
#endif
    auto time_start = std::chrono::steady_clock::now();
    Factory<TaskData> task_factory(3);
    Factory<VisionData> data_transmit_factory(5);  //visiondata
    MessageFilter<MCUData> data_receiver(100);
    serial serial(serial_name,BAUD);

#ifdef USING_IMU_C_BOARD
    std::thread serial_watcher(&serialWatcher,ref(serial));
    printf("串口监视线程serialWatcher start!");
    std::thread receiver(&dataReceiver,ref(serial),ref(data_receiver),time_start);
    printf("串口接收线程receiver start!");
#endif

    /**
     * 生产者线程
     **/
    std::thread task_producer(&producer,ref(task_factory),ref(data_receiver),time_start);
    printf("生产者线程task_producer start!");

    /**
     * 消费者线程
     * */
     std::thread task_consumer(&consumer,ref(task_factory),ref(data_transmit_factory));
     printf("消费者线程task_consumer start!");

     /**
      * 数据发送线程
      **/
      std::thread transmitter(&dataTransmitter,ref(serial),ref(data_transmit_factory));
      printf("数据发送线程transmitter start!");

      task_producer.join();
      printf("生产者线程task_producer end!");

      task_consumer.join();
      printf("消费者线程task_consumer end!");

#ifdef USING_IMU_C_BOARD
      serial_watcher.join();
      printf("串口监视线程serial_watcher end!");
#endif

      transmitter.join();
      printf("串口发送线程transmitter end!");

#ifdef USING_IMU_C_BOARD
      receiver.join();
#endif

      return 0;

}