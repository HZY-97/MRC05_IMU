#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include <arpa/inet.h>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <linux/sockios.h>
#include <memory>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unistd.h>

#define GYRO_DEV "/dev/ttyS4"
// #define	GYRO_DEV   "/dev/ttyUSB1"
const unsigned char cali_offset[] = {0xFF, 0x1C, 0, 0, 0, 0, 0xC5, 0xD6};
const unsigned char cmd_stop[] = {0xFF, 7, 0, 6, 0, 0, 0x41, 0xD5};
const unsigned char cmd_start[] = {0xFF, 0x7, 0, 0, 0, 0, 0xa1, 0xD4};
const unsigned char clrear_yaw[] = {0xFF, 0x1e, 0, 0, 0, 0, 0xbc, 0x16};

int set_opt(int, int, int, char, int);
unsigned char buf[512];

class ImuPub : public rclcpp::Node {
public:
  ImuPub() : Node("imu_data_pub_node") {
    m_pubImu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 1000);
  }

public:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pubImu;
};
int main(int argc, char **argv) {

  rclcpp::InitOptions rcl_options;
  rcl_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, rcl_options);

  std::shared_ptr<ImuPub> node = std::make_shared<ImuPub>();

  int fd, nByte, flag = 1;
  short sum_1;
  int cnt = 0;
  int err = 0;
  char dev[16];
  int select;
  int custom_baud;
  int product;
  int parameter;
  int sampling_clock;
  sensor_msgs::msg::Imu imu_data;

  unsigned char buffer[512];

  memset(buffer, 0, sizeof(buffer));
  // if((fd = open(GYRO_DEV, O_RDWR|O_NOCTTY))<0)//默认为阻塞读方式
  if ((fd = open(GYRO_DEV, O_RDWR | O_NONBLOCK)) < 0) //非阻塞读方式

    printf("open %s is failed\n", GYRO_DEV);
  else {

    fcntl(fd, F_SETFL, 0); // set zuse
    set_opt(fd, 115200, 8, 'N', 1);

    usleep(1000);
    printf("write calibration\n");
    write(fd, cmd_stop, 8);
    usleep(1000);
    write(fd, cmd_start, 8);
    usleep(1000);
    write(fd, cali_offset, 8); //开启时候发送该命令进行漂移补偿
                               //车体必须保持静止3s，status为0b表示补偿完成
    printf("rd\n");
    std::chrono::milliseconds interval(10);
    while (rclcpp::ok()) // && ser.available())
    {
      nByte = read(fd, buffer, 1);
      if (nByte > 0) {
        cnt += nByte;
        for (int i = 0; i < nByte; i++) {
          buf[i + cnt] = buffer[i];
          // printf("%02x,", buffer[i]); //可以打开打印
          // ff,ff,00,00,0b,00,00,00,00,00,01,00,15,00,f0,03,10,00,3b,21,22,16,36,ce,56,14,26,03,0d,0a
          /*2个ff 固定头，0d 0a固定尾巴，中间short型分别为，计数，状态，gyrox
          gyroy gryoz，
          accx，accy，accz，pitch，roll，yaw，temperature，checksum*/
        }
        nByte = 0;
      }
      if (cnt >= 30) {
        for (int i = 0; i < 30; i++) {
          if (buf[0 + i] == 0x80 && buf[1 + i] == 0x01 && buf[28 + i] == 0xd &&
              buf[29 + i] == 0xa) {
            short sta;
            short sum = 0;
            for (int j = 2; j < 26; j++)
              sum += buf[i + j];
            sum_1 = (buf[i + 27] << 8) + buf[i + 26];
            if (sum == sum_1) {
              float yaw, pich, roll;
              float acc_x, acc_y, acc_z;
              float grox_x, grox_y, grox_z;
              float temperature;
              short val = (buf[23 + i] << 8) + buf[i + 22];
              yaw = (float)(val) / 100;
              val = (buf[21 + i] << 8) + buf[i + 20];
              roll = (float)(val) / 100;
              val = (buf[19 + i] << 8) + buf[i + 18];
              pich = (float)(val) / 100;

              val = (buf[17 + i] << 8) + buf[i + 16];
              acc_z = (float)(val) / 100;
              val = (buf[15 + i] << 8) + buf[i + 14];
              acc_y = (float)(val) / 100;
              val = (buf[13 + i] << 8) + buf[i + 12];
              acc_x = (float)(val) / 100;

              val = (buf[11 + i] << 8) + buf[i + 10];
              grox_z = (float)(val) / 64 * M_PI / 180.0; // 100
              val = (buf[9 + i] << 8) + buf[i + 8];
              grox_y = (float)(val) / 64 * M_PI / 180.0;
              val = (buf[7 + i] << 8) + buf[i + 6];
              grox_x = (float)(val) / 64 * M_PI / 180.0;

              val = (buf[25 + i] << 8) + buf[i + 24];
              temperature = (float)(val) / 100;

              imu_data.header.stamp = rclcpp::Clock().now();
              imu_data.header.frame_id = "map";
              //加速度
              imu_data.linear_acceleration.x = acc_x;
              imu_data.linear_acceleration.y = acc_y;
              imu_data.linear_acceleration.z = acc_z;
              //角速度
              imu_data.angular_velocity.x = grox_x;
              imu_data.angular_velocity.y = grox_y;
              imu_data.angular_velocity.z = grox_z;
              //姿态四元数
              geometry_msgs::msg::Quaternion q;
              roll = roll * M_PI / 180.0;
              pich = pich * M_PI / 180.0;
              yaw = yaw * M_PI / 180.0;

              imu_data.orientation_covariance[0] = yaw;

              // q = tf::createQuaternionMsgFromRollPitchYaw(roll, pich, yaw);
              // imu_data.orientation.w = q.w;
              // imu_data.orientation.x = q.x;
              // imu_data.orientation.y = q.y;
              // imu_data.orientation.z = q.z;
              //发布topic
              node->m_pubImu->publish(imu_data);

              i += 30;

              // std::cout<<ros::Time::now()<<std::endl;
              // std::cout<<"grox_x:"<<grox_x<<" "<<"grox_y:"<<grox_y<<"
              // "<<"grox_z:"<<grox_z<<std::endl;
              // std::cout<<"acc_x:"<<acc_x<<" "<<"acc_y:"<<acc_y<<"
              // "<<"acc_z:"<<acc_z<<std::endl;
              std::cout << "pich:" << pich << " "
                        << "roll:" << roll << " "
                        << "yaw:" << yaw << std::endl;
              // std::cout<<"temperature:"<<temperature<<std::endl;
              for (int k = 0; k < 30; k++) {
                buf[i + k] = 0;
              }
            } else
              err++;

            cnt = 0;
          }
        }
      }
      if (cnt >= 512)
        cnt = 0;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(interval);
  }
  rclcpp::shutdown();
  return 0;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
  struct termios newtio, oldtio;
  if (tcgetattr(fd, &oldtio) != 0) {
    perror("SetupSerial 1");
    return -1;
  }
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  switch (nBits) {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }

  switch (nEvent) {
  case 'O':
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'E':
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'N':
    newtio.c_cflag &= ~PARENB;
    break;
  }

  switch (nSpeed) {
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
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;
  newtio.c_cc[VTIME] = 100; ///* 设置超时10 seconds*/
  newtio.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);
  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
    perror("com set error");
    return -1;
  }

  //	printf("set done!\n\r");
  return 0;
}
