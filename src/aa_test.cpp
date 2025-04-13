#include "aa_test.hpp"

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

    autoaim_pub2_ = this->create_publisher<std_msgs::msg::Float32>("shoot_info2", 10);

    this->rand_seed1 = (double)rand() / RAND_MAX;
    this->rand_seed2 = (double)rand() / RAND_MAX;
    this->rand_seed3 = (double)rand() / RAND_MAX;
}

AA_Test::~AA_Test() {
    RCLCPP_INFO(this->get_logger(), "AA_Test node stopped");
}

void AA_Test::publish_autoaim() {
    send_count_++;
    cal_angle();
    autoaim_msg_ = std::make_shared<communicate_2025_aatest::msg::Autoaim>();
    autoaim_msg_->pitch = this->pitch_;
    autoaim_msg_->high_gimbal_yaw = this->yaw_;
    autoaim_pub_->publish(*autoaim_msg_);

    autoaim_msg2_ = std::make_shared<std_msgs::msg::Float32>();
    autoaim_msg2_->data = this->pitch_;
    autoaim_pub2_->publish(*autoaim_msg2_);

    RCLCPP_INFO(
        this->get_logger(),
        "Published autoaim message: pitch=%f, yaw=%f",
        autoaim_msg_->pitch,
        autoaim_msg_->high_gimbal_yaw
    );
}

void AA_Test::cal_angle() {
    Target_config config = this->config_;
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
            if (fmod(time + phase, period) < period / 2) {
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

            return offset + amplitude * (fmod(time, period) / period);
        };
    } else if (config.wave == SINEWAVE) {
        config.wave_func = [time, config]() {
            double period = config.period;
            double amplitude = config.amplitude;
            double offset = config.offset;

            return offset + amplitude * sin(2 * M_PI * fmod(time, period) / period);
        };
    } else if (config.wave == TRIANGLEWAVE) {
        config.wave_func = [time, config]() {
            double period = config.period;
            double amplitude = config.amplitude;
            double offset = config.offset;

            return offset + amplitude * (1 - fabs(fmod(time / period + 0.5, 1) - 0.5) * 4);
        };
    }

    else if (config.wave == RANDOM)
    {
        config.wave_func = [this, config]() {
            double amplitude = config.amplitude;

            double rand_value = ((double)rand() / RAND_MAX) * 2 - 1; // 生成[-1, 1]之间的随机数

            double last_data =
                (config.target == AUTOAIM_PITCH) ? this->pitch_.load() : this->yaw_.load();

            if (config.period == 0) {
                return last_data;
            }

            if (this->send_count_ % int(config.period / 10) == 0) {
                return amplitude * rand_value + last_data;

            } else {
                return last_data;
            }

            return amplitude * rand_value + last_data;
        };
    } else if (config.wave == SINEWAVES) {
        config.wave_func = [this, config]() {
            double amplitude = config.amplitude;
            double offset = config.offset;
            double phase = config.phase;

            double sin1 = offset
                + amplitude * 3
                    * sin(
                        2 * M_PI * fmod(this->send_count_, config.period) / config.period
                            * this->rand_seed1
                        + phase
                    );
            double sin2 = offset
                + amplitude * 6
                    * sin(
                        1 * M_PI * fmod(this->send_count_, config.period) / config.period
                            * this->rand_seed2
                        + phase
                    );
            double sin3 = offset
                + amplitude * 2
                    * sin(
                        3 * M_PI * fmod(this->send_count_, config.period) / config.period
                            * this->rand_seed3
                        + phase
                    );

            return (sin1 + sin2 + sin3) / 11;
        };
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid wave type: %d", config.wave);
        return;
    }

    if (config.wave != NONE) {
        temp = config.wave_func();
    }

    if(config.noise_enable) {
        double noise = ((double)rand() / RAND_MAX) * 2 - 1; // 生成[-1, 1]之间的随机数
        temp += config.noise_amplitude * noise;
    }

    // 处理角度范围
    if (config.target == AUTOAIM_PITCH || config.target == AUTOAIM_YAW) {
        temp = fmod(temp + M_PI, 2 * M_PI);
        if (temp < 0) {
            temp += 2 * M_PI;
        }
        temp -= M_PI;
    }

    if (config.target == AUTOAIM_PITCH) {
        this->pitch_.store(temp);
    } else if (config.target == AUTOAIM_YAW) {
        this->yaw_.store(temp);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid target: %d", config.target);
    }
    return;
}

void AA_Test::set(Target_config config) {
    // config.wave = SQUAREWAVE;
    // 计算目标值

    this->config_ = config;
    return;
}