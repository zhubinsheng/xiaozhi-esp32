#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include "config.h"
#include <driver/i2c_master.h>
#include <memory>
#include "mpu6050_sensor.h"

class ImuManager {
public:
    static ImuManager& GetInstance();
    
    // 初始化IMU系统
    bool Initialize();
    
    // 启动/停止IMU任务
    void StartImuTask();
    void StopImuTask();
    
    // 检查是否已初始化
    bool IsInitialized() const { return initialized_; }

    // 配置常量（集中管理魔法数）
    struct ImuDetectConfig {
        static constexpr float kAngleChangeThresholdDeg = 1.0f;   // 姿态角变化阈值
        static constexpr uint32_t kInitWarmupSamples = 30;         // 初始化样本数
        static constexpr uint32_t kCalibrationSamples = 100;       // 占位校准样本数
        static constexpr uint32_t kTaskIntervalMs = 100;           // 任务循环间隔
        static constexpr uint32_t kMinuteMs = 10000;               // 一分钟毫秒数
        static constexpr float kMinuteStaticRatio = 0.90f;         // 当分钟静止比例阈值
        static constexpr uint32_t kConsecutiveStaticMinutesForSleep = 2; // 连续静止分钟数阈值
    };

private:
    ImuManager() = default;
    ~ImuManager() = default;
    ImuManager(const ImuManager&) = delete;
    ImuManager& operator=(const ImuManager&) = delete;
    
    void InitializeImu();
    static void ImuDataTask(void *pvParameters);
    
    bool initialized_ = false;
    i2c_master_bus_handle_t imu_i2c_bus_ = nullptr;
    std::unique_ptr<Mpu6050Sensor> mpu6050_sensor_;
    
    // 任务句柄
    TaskHandle_t imu_task_handle_ = nullptr;
};

#endif // IMU_MANAGER_H
