#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "communicate_2025_aatest/msg/autoaim.hpp"

// 目标
enum { AUTOAIM_YAW = 0, AUTOAIM_PITCH };

// 预设波形
enum {
    NONE = 0,     // 无波形
    SQUAREWAVE,   // 方波
    SAWTOOTH,     // 锯齿波
    SINEWAVE,     // 正弦波
    TRIANGLEWAVE, // 三角波
    RANDOM,       // 随机波
    SINEWAVES     // 多正弦波叠加
};

struct Target_config {
    int target = AUTOAIM_YAW;          // 目标
    double value = 0;                  // 设定固定值
    int wave = NONE;                   // 波形
    double period;                     // 周期
    double amplitude;                  // 振幅
    double offset;                     // 振幅偏移量
    double phase;                      // 相位
    double max;                        // 最大值
    double min;                        // 最小值
    std::function<double()> wave_func; // 波形生成函数
    int noise_enable;                  // 噪声是否启动
    double noise_amplitude;            // 噪声强度
};

class AA_Test: public rclcpp::Node {
public:
    AA_Test(std::string node_name = "AA_Test");
    ~AA_Test();

    void show();

    /**
     * @brief 接收目标设定
     * 
     * @param config 参数
     */
    void set(Target_config config);

private:
    /**
     * @brief 发布消息
     * 
     */
    void publish_autoaim();

    void cal_angle();

    rclcpp::Publisher<communicate_2025_aatest::msg::Autoaim>::SharedPtr autoaim_pub_;
    int send_count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<communicate_2025_aatest::msg::Autoaim> autoaim_msg_;

    // double pitch_;
    // double yaw_;

    Target_config config_;

    std::atomic<double> pitch_;
    std::atomic<double> yaw_;

    std::deque<double> pitch_history_;
    std::deque<double> yaw_history_;

    double publish_rate_ = 3.0;

    double max_pitch_ = 0.0;
    double min_pitch_ = 0.0;

    double max_yaw_ = -1;
    double min_yaw_ = -1;

    double rand_seed1;
    double rand_seed2;
    double rand_seed3;
};