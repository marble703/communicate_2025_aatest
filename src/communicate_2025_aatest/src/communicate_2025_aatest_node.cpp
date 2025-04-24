#include "aa_test.hpp"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

/**
 * @brief 创建一个窗口和滑块
 * 
 * @param window_name 窗口名
 * @param bar_name 滑块名
 * @param value 值，int类型
 * @param max_value 最大值
 */
void createBarwithWindow(
    const std::string& window_name,
    const std::string& bar_name,
    int* value,
    int max_value
) {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::createTrackbar(bar_name, window_name, value, max_value);
    cv::setTrackbarPos(bar_name, window_name, *value);
}

/**
 * @brief 创建一个滑块
 * 
 * @param window_name 窗口名
 * @param bar_name 滑块名
 * @param value 值，int类型
 * @param max_value 最大值
 */
void createBaronWindow(
    const std::string& window_name,
    const std::string& bar_name,
    int* value,
    int max_value
) {
    cv::createTrackbar(bar_name, window_name, value, max_value);
    cv::setTrackbarPos(bar_name, window_name, *value);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 初始化参数，使用opencv滑块
    Target_config config;
    int target = AUTOAIM_YAW; // 目标
    int value = 314;          // 设定固定值,从0到628映射到-3.14到3.14
    int wave = NONE;          // 波形
    int period = 10;          // 周期，单位100ms
    int amplitude = 45;       // 振幅，单位0.01
    int offset = 314;         // 振幅偏移量，从0到628到100映射到-3.14到3.14
    int phase = 0;            // 相位，从0到628到100映射到-3.14到3.14
    int noise_enable = 0;     // 噪声是否启动
    int noise_amplitude = 1;  // 噪声强度
    // int max = 45 / 180 / 3.14;  // 最大值，从0到100映射到-3.14到3.14
    // int min = -15 / 180 / 3.14; // 最小值，从0到100映射到-3.14到3.14

    createBarwithWindow("Autoaim_control", "target", &target, 1);
    createBaronWindow("Autoaim_control", "value", &value, 628);
    createBaronWindow("Autoaim_control", "wave", &wave, SINEWAVES);
    createBaronWindow("Autoaim_control", "period", &period, 100);
    createBaronWindow("Autoaim_control", "amplitude", &amplitude, 628);
    createBaronWindow("Autoaim_control", "offset", &offset, 628);
    createBaronWindow("Autoaim_control", "phase", &phase, 628);
    createBaronWindow("Autoaim_control", "noise_enable", &noise_enable, 1);
    createBaronWindow("Autoaim_control", "noise_amplitude", &noise_amplitude, 314);

    // createBaronWindow("Autoaim_control", "max", &max, 180);
    // createBaronWindow("Autoaim_control", "min", &min, 180);

    auto node = std::make_shared<AA_Test>();

    std::thread set_config_thread([&]() {
        while (rclcpp::ok()) {
            config.target = target;
            config.value = double(value) / 100.0 - 3.14;
            config.wave = wave;
            config.period = period * 100.0;
            config.amplitude = amplitude / 100.0;
            config.offset = offset / 100.0 - 3.14;
            config.phase = phase / 100.0 - 3.14;
            config.noise_enable = noise_enable;
            config.noise_amplitude = noise_amplitude / 100.0;
            // config.max = max / 100.0 * 3.14 - 3.14;
            // config.min = min / 100.0 * 3.14 - 3.14;
            node->set(config);
            cv::waitKey(100);
        }
    });

    set_config_thread.detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}