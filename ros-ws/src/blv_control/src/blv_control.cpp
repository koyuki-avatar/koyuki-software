#include <chrono>
#include <cstdio>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "blvr_comunicator.h"

class BlvController : public rclcpp::Node {
public:
    BlvController(): Node("blv_control") {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&BlvController::cmd_vel_callback, this, std::placeholders::_1));
        loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&BlvController::loop, this));

        this->declare_parameter("wheel_radius", 0.2); // [m]
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        this->declare_parameter("wheel_distance", 0.485); // [m]
        wheel_distance_ = this->get_parameter("wheel_distance").as_double();
        this->declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
        serial_port_ = this->get_parameter("serial_port").as_string();

        
        comm_ = std::make_shared<BlvrComunicator>();

        linear_x_now = 0.0f;
        angular_z_now = 0.0f;

        comm_->openDevice(serial_port_);
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool loop();

private:
    void calc_rpm(int* rpm_r, int* rpm_l, const double linear_x, const double angular_z);

    double wheel_radius_;
    double wheel_distance_;
    std::string serial_port_;

    double linear_x_now;
    double angular_z_now;
    std::shared_ptr<BlvrComunicator> comm_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    bool is_ex_set = false;
};

void BlvController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    linear_x_now = msg->linear.x;
    angular_z_now = msg->angular.z;
    //RCLCPP_INFO(this->get_logger(), "cmd_vel: %f %f", linear_x_now, angular_z_now); // OK
    // this->get_clock()->now(); // time
}


bool BlvController::loop() {
    // control motor
    int rpm[2] = {0};
    this->calc_rpm(&rpm[0], &rpm[1], linear_x_now, angular_z_now);

//    RCLCPP_INFO(this->get_logger(), "rpm: %d %d", rpm[0], rpm[1]);
    if (!is_ex_set || (comm_->comm_err_cnt>0)) {
        is_ex_set = true;
        for (uint8_t i = 0; i < 2; i++) {
            if (comm_->setExcitation(i+1) != 0) {
                fprintf(stderr, "cannot set exicitation");
                is_ex_set = false;
                return false;
            }
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        comm_->directDataDrive(i+1, BlvrComunicator::MOTION_CONTINUOUS_OPERATION_BY_RPM, 0, rpm[i], 500, 500, 1000);
    }
    return true;
}

//private
void BlvController::calc_rpm(int* rpm_r, int* rpm_l, const double linear_x, const double angular_z) {
    double tmp[2] = {0.0f};
    // Right
    tmp[0] = -((linear_x + wheel_distance_ * angular_z * 0.5) / (2.0f * M_PI * wheel_radius_));
    // Left
    tmp[1] = ((linear_x - wheel_distance_ * angular_z * 0.5) / (2.0f * M_PI * wheel_radius_));

    *rpm_r = int(tmp[0]*30.0*60.0);
    *rpm_l = int(tmp[1]*30.0*60.0);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<BlvController> node = std::make_shared<BlvController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
