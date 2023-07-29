#define TEENSY_PORT "/dev/ttyACM0"
#include "triton_controls/serial_subscriber.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
using std::placeholders::_1;

namespace triton_controls
{

  SerialSubscriber::SerialSubscriber(const rclcpp::NodeOptions & options)
  : Node("serial_subscriber", options)
  {
    fd_ = open(TEENSY_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    struct termios tty;
    tcgetattr (fd_, &tty);
    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);
    
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    thruster_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
      "motor_control", 10, std::bind(&SerialSubscriber::controlCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Serial Subscriber succesfully started!");
  }

  SerialSubscriber::~SerialSubscriber(){
    close(fd_);
  }

  void SerialSubscriber::controlCallback(const std_msgs::msg::UInt32::SharedPtr msg) const
  {
    //RCLCPP_INFO(this->get_logger(), "WRITING SERIAL");
    std::cout << "Serial subscriber writing message: " << msg->data << std::endl;
    write(fd_, &msg->data, 4);
  }

} // namespace triton_controls