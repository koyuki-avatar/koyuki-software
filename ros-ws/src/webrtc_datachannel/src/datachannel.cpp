
extern "C" {
#include <time.h>
#include <termios.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>
#include <stdlib.h>
}

#include <string>
#include <cstring>
#include <cmath>
#include <regex>

#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define BUFSIZE 128

class DataChannel : public rclcpp::Node {
public:
    DataChannel(): Node("webrtc_datachannel") {
        this->declare_parameter("serial_port", std::string("/dev/pts/3"));
        serial_port_name_ = this->get_parameter("serial_port").as_string();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DataChannel::loop, this));

        is_open = false;
        openPort(serial_port_name_);
    }

private:
    static const int BAUDRATE = B115200;
    bool openPort(std::string& serial_port);
    bool loop();
    void pubMsg(const std::string& msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    std::string serial_port_name_;
    int port;
    bool is_open;
    char buf[BUFSIZE];

    time_t last_rcv;
};

bool DataChannel::loop() {
    memset(buf, '\0', BUFSIZE);
    int ret = read(port, buf, BUFSIZE);
    time_t tnow = time(NULL);
    if (ret > 0) {
        last_rcv = time(NULL);
        pubMsg(this->buf);
    } else if (ret < 0) {
        fprintf(stderr,"%s:%s:%d: read error\n",
                      __FILE__, __func__, __LINE__);
    }
    if ((tnow-last_rcv)>=2) {
        pubMsg("{\"x\":0,\"z\":0");
    }
    return true;
}

void DataChannel::pubMsg(const std::string& msg) {
    double x=0, z=0;
    auto pubmsg = geometry_msgs::msg::Twist();
    std::string nmsg = std::regex_replace(msg, std::regex("\n"), "");
    printf("%s\n", nmsg.data());
    int ret = std::sscanf(nmsg.data(), "{\"x\":%lf,\"z\":%lf}", &x, &z);
    if (ret < 0) {
        fprintf(stderr,"%s:%s:%d: sscanf error\n", __FILE__, __func__, __LINE__);
    } else {
        printf("x: %f, z: %f\n", x, z);
    }
    pubmsg.linear.x = x;
    pubmsg.angular.z = z;
    cmd_vel_pub_->publish(pubmsg);
}

bool DataChannel::openPort(std::string& serial_port) {

  if ((port = open(serial_port.c_str(), O_RDWR | O_NONBLOCK)) < 0) {
    fprintf(stderr, "%s: can't open port(%s)\n", __func__, serial_port.c_str());
    return false;
  }

  struct termios p;
  tcgetattr(port, &p);

  memset(&p, 0, sizeof (struct termios));
  p.c_iflag = IGNPAR;
  p.c_cflag = BAUDRATE | CS8 | CREAD | CLOCAL;
//  p.c_lflag = p.c_lflag & ~ICANON;

  tcsetattr(port, TCSAFLUSH, const_cast<const termios*>(&p));

  is_open = true;

  return true;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataChannel>());
    rclcpp::shutdown();

    return 0;
}
