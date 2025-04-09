#include "aa_test.hpp"
#include <iostream>

AA_Test::AA_Test(std::string node_name): Node(node_name) {
    RCLCPP_INFO(this->get_logger(), "AA_Test node started");

    // 读取参数
    this->publish_rate_ = this->declare_parameter("publish_rate", 3.0);
    this->max_pitch_ = this->declare_parameter("pitch_range/max", 3.0);
    this->min_pitch_ = this->declare_parameter("pitch_range/min", -3.0);
    this->max_yaw_ = this->declare_parameter("yaw_range/max", 3.0);
    this->min_yaw_ = this->declare_parameter("yaw_range/min", -3.0);

    autoaim_pub_ = this->create_publisher<communicate_2025_aatest::msg::Autoaim>("shoot_info", 10);

    this->pitch_ = 0.0;
    this->yaw_ = 0.0;

    // 每3ms发布一次
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(3),
        std::bind(&AA_Test::publish_autoaim, this)
    );
}

AA_Test::~AA_Test() {
    RCLCPP_INFO(this->get_logger(), "AA_Test node stopped");
}

void AA_Test::publish_autoaim() {
    send_count_++;
    autoaim_msg_ = std::make_shared<communicate_2025_aatest::msg::Autoaim>();
    autoaim_msg_->pitch = this->pitch_;
    autoaim_msg_->high_gimbal_yaw = this->yaw_;
    autoaim_pub_->publish(*autoaim_msg_);

    RCLCPP_INFO(
        this->get_logger(),
        "Published autoaim message: pitch=%f, yaw=%f",
        autoaim_msg_->pitch,
        autoaim_msg_->high_gimbal_yaw
    );
}

std::map<int, double> AA_Test::cal_angle() {}

void AA_Test::set(Target_config config) {
    // config.wave = SQUAREWAVE;
    // 计算目标值

    double time = this->send_count_ * this->publish_rate_;
    std::cout << "time: " << time << std::endl;

    double temp;
    if (config.wave == NONE) {
        temp = config.value;

    } else if (config.wave == SQUAREWAVE) {
        config.wave_func = [time, config]() {
            double period = config.period;
            double amplitude = config.amplitude;
            double offset = config.offset;
            double phase = config.phase;
            if (time + phase / period < period / 2) {
                return offset + amplitude;
            } else {
                return offset - amplitude;
            }
        };
    } else if (config.wave == SAWTOOTH) {
        config.wave_func = [time, config]() {
            double period = config.period;
            double amplitude = config.amplitude;
            double offset = config.offset;

            return offset + amplitude * (time / period);
        };
    } else if (config.wave == SINEWAVE) {
        config.wave_func = [time, config]() {
            double period = config.period;
            double amplitude = config.amplitude;
            double offset = config.offset;

            return offset + amplitude * sin(2 * M_PI * time / period);
        };
    } else if (config.wave == TRIANGLEWAVE) {
        config.wave_func = [time, config]() {
            double period = config.period;
            double amplitude = config.amplitude;
            double offset = config.offset;

            return offset + amplitude * (1 - fabs(fmod(time / period + 0.5, 1) - 0.5) * 4);
        };
    }

    if (config.wave != NONE) {
        temp = config.wave_func();
    }

    // 处理范围
    if (config.target == AUTOAIM_PITCH) {
        if (max_pitch_ != -1 && temp > this->max_pitch_) {
            temp = this->max_pitch_;
            RCLCPP_WARN(this->get_logger(), "Pitch exceeds max limit: %f", this->max_pitch_);
        } else if (min_pitch_ != -1 && temp < this->min_pitch_) {
            temp = this->min_pitch_;
            RCLCPP_WARN(this->get_logger(), "Pitch exceeds min limit: %f", this->min_pitch_);
        }
        this->pitch_.store(temp);
    } else if (config.target == AUTOAIM_YAW) {
        if (max_yaw_ != -1 && temp > this->max_yaw_) {
            temp = this->max_yaw_;
            RCLCPP_WARN(this->get_logger(), "Yaw exceeds max limit: %f", this->max_yaw_);
        } else if (min_yaw_ != -1 && temp < this->min_yaw_) {
            temp = this->min_yaw_;
            RCLCPP_WARN(this->get_logger(), "Yaw exceeds min limit: %f", this->max_yaw_);
        }

        this->yaw_.store(temp);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid target: %d", config.target);
    }
    return;
}