#include "serial.h"

/**
 * 获取串口名称
 * **/
string get_uart_dev_name() {
    FILE *ls = popen("ls /dev/ttyUSB* --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    return name;
}

serial::serial(const string name, const int BAUD)
{
    serial_name= name;
    baud = BAUD;  //波特
    InitPort();
}

/**
 * 设置
 * speed     波特率
 * parity    校验位
 * databits  数据位
 * stopbits  停止位
 * */
int serial::set_opt(int fd, int speed, char parity, int databits, int stopbits) {
    termios newtio{}, oldtio{};
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (databits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            break;
    }

    switch (parity) {
        case 'O':  //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':  //无校验
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }

    switch (speed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 921600:
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if (stopbits == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (stopbits == 2) {
        newtio.c_cflag |= CSTOPB;
    }

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");

    return 0;
}


/**
 *@brief   获取模式命令
 */
bool serial::get_Mode()
{
    int bytes;
    char *name = ttyname(fd);
    if (name == NULL) printf("tty is null\n");
    int result = ioctl(fd, FIONREAD, &bytes);
    if (result == -1)
        return false;

    if (bytes == 0)
    {
        //    cout << "缓冲区为空" << endl;
        return false;
    }
    // bytes = read(fd, rdata, 49);
    bytes = read(fd, rdata, 58);  //56位数据+2校验位
    // bytes = read(fd, rdata, 45);
    // cout<<bytes<<endl;

    if (rdata[0] == 0xA5 && Verify_CRC8_Check_Sum(rdata, 3))
    {
        mode = rdata[1];
        getQuat(&rdata[3]);
        getGyro(&rdata[19]);
        getAcc(&rdata[31]);
        getSpeed(&rdata[43]);
        Verify_CRC16_Check_Sum(rdata,50);
        //TODO:接收下位机发送的弹速
    }
    return true;

}

/**
 *@brief   初始化数据
 *@param  fd       类型  int  打开的串口文件句柄
 *@param  speed    类型  int  波特率

 *@param  databits 类型  int  数据位   取值 为 7 或者8

 *@param  stopbits 类型  int  停止位   取值为 1 或者2
 *@param  parity   类型  int  效验类型 取值为N,E,O,S
 *@param  portchar 类型  char* 串口路径
 */
bool serial::InitPort(int speed, char parity, int databits, int stopbits) {
    string name = get_uart_dev_name();
    if (name == "") {
        return false;
    }
    if ((fd = open(name.data(), O_RDWR)) < 0) {
        return false;
    }
    if (set_opt(fd, speed, parity, databits, stopbits) < 0) {
        return false;
    }
    fmt::print("✔️ ✔️ ✔️ 🌈 恭喜你串口插了 🌈 ✔️ ✔️ ✔️\n");
    return true;
}

/**
 *@brief   转换数据并发送
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
void serial::TransformData(const VisionData &data)
{

    Tdata[0] = 0xD4;

    Tdata[1] = CmdID1;
    Append_CRC8_Check_Sum(Tdata, 3);

    Tdata[3] = data.pitch_angle.c[0];
    Tdata[4] = data.pitch_angle.c[1];
    Tdata[5] = data.pitch_angle.c[2];
    Tdata[6] = data.pitch_angle.c[3];

    Tdata[7] = data.yaw_angle.c[0];
    Tdata[8] = data.yaw_angle.c[1];
    Tdata[9] = data.yaw_angle.c[2];
    Tdata[10] = data.yaw_angle.c[3];

    Tdata[11] = data.dis.c[0];
    Tdata[12] = data.dis.c[1];
    Tdata[13] = data.dis.c[2];
    Tdata[14] = data.dis.c[3];

    Tdata[15] = data.oppo_speed.c[0];
    Tdata[16] = data.oppo_speed.c[1];
    Tdata[17] = data.oppo_speed.c[2];
    Tdata[18] = data.oppo_speed.c[3];

    Tdata[19] = data.oppo_acc.c[0];
    Tdata[20] = data.oppo_acc.c[1];
    Tdata[21] = data.oppo_acc.c[2];
    Tdata[22] = data.oppo_acc.c[3];

    Tdata[23] = data.isSpinning;
    Tdata[24] = data.shoot_flag;
    Tdata[25] = 0x00;

    Append_CRC16_Check_Sum(Tdata, 26);

}

/////////////////////////////////////////////
/**
 * @brief 将4个uchar转换为float
 *
 * @param data data首地址指针
 * @return
 */
float serial::exchange_data(unsigned char *data)
{
    float float_data;
    float_data = *((float*)data);
    return float_data;
};

/**
 * @brief 解算四元数数据
 *
 * @param data 四元数首地址指针
 * @return
 */
bool serial::getQuat(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];
    unsigned char* f4 = &data[12];

    quat[0] = exchange_data(f1);
    quat[1] = exchange_data(f2);
    quat[2] = exchange_data(f3);
    quat[3] = exchange_data(f4);
    // fmt::print(fmt::fg(fmt::color::white), "quat: {} {} {} {} \n", quat[0], quat[1], quat[2], quat[3]);
    return true;
}

/**
 * @brief 解算角速度数据
 *
 * @param data 角速度首地址指针
 * @return
 */
bool serial::getGyro(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    gyro[0] = exchange_data(f1);
    gyro[1] = exchange_data(f2);
    gyro[2] = exchange_data(f3);

    // fmt::print(fmt::fg(fmt::color::white), "gyro: {} {} {} \n", gyro[0], gyro[1], gyro[2]);
    return true;
}

/**
 * @brief 解算加速度数据
 *
 * @param data 加速度首地址指针
 * @return
 */
bool serial::getAcc(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    acc[0] = exchange_data(f1);
    acc[1] = exchange_data(f2);
    acc[2] = exchange_data(f3);
    // fmt::print(fmt::fg(fmt::color::white), "acc: {} {} {} \n", acc[0], acc[1], acc[2]);
    return true;
}
/**
 * @brief 解算速度数据
 *
 * @param data 速度首地址指针
 * @return
 */
bool serial::getSpeed(unsigned char *data)
{
    unsigned char* f1 = &data[0];

    bullet_speed = exchange_data(f1);

    // fmt::print(fmt::fg(fmt::color::white), "speed: {} \n", bullet_speed);
    return true;
}
//////////////////////////////////////////////

//发送数据函数
void serial::send()
{
    auto write_stauts = write(fd, Tdata, 30);
}

//关闭通讯协议接口
void serial::closePort()
{
    close(fd);
}

