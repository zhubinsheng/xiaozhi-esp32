#include "imu_manager.h"
#include <esp_log.h>
#include <cmath>
#include "protocols/sleep_music_protocol.h"
#include "board.h"

#define TAG "ImuManager"

ImuManager& ImuManager::GetInstance() {
    static ImuManager instance;
    return instance;
}

bool ImuManager::Initialize() {
    if (initialized_) {
        ESP_LOGW(TAG, "ImuManager already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing ImuManager...");
    
    InitializeImu();
    // 不在初始化时自启任务，由上层(ADC躺下检测)控制 Start/Stop
    initialized_ = true;
    ESP_LOGI(TAG, "ImuManager initialized successfully");
    return true;
}

void ImuManager::InitializeImu() {
    ESP_LOGI(TAG, "Initializing MPU6050 IMU sensor...");

    // 使用 driver_ng 创建 I2C 总线并初始化自研传感器封装
    i2c_master_bus_config_t imu_i2c_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = IMU_I2C_SDA_PIN,
        .scl_io_num = IMU_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = false,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&imu_i2c_cfg, &imu_i2c_bus_));

    mpu6050_sensor_ = std::make_unique<Mpu6050Sensor>(imu_i2c_bus_);
    if (mpu6050_sensor_) {
        uint8_t device_id;
        if (mpu6050_sensor_->GetDeviceId(&device_id)) {
            ESP_LOGI(TAG, "MPU6050 device ID: 0x%02X", device_id);
            if (device_id == MPU6500_WHO_AM_I_VAL) {
                if (mpu6050_sensor_->Initialize(ACCE_FS_4G, GYRO_FS_500DPS)) {
                    if (mpu6050_sensor_->WakeUp()) {
                        initialized_ = true;
                        ESP_LOGI(TAG, "MPU6050 sensor initialized successfully");
                    } else {
                        ESP_LOGE(TAG, "Failed to wake up MPU6050");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to initialize MPU6050");
                }
            } else {
                ESP_LOGE(TAG, "Unsupported IMU device ID: 0x%02X (expected 0x%02X)", device_id, MPU6500_WHO_AM_I_VAL);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read MPU6050 device ID");
        }
    } else {
        ESP_LOGE(TAG, "Failed to create MPU6050 sensor instance");
    }

    if (!initialized_) {
        ESP_LOGW(TAG, "IMU sensor initialization failed - continuing without IMU");
    }
}

void ImuManager::StartImuTask() {
    if (!initialized_) {
        ESP_LOGW(TAG, "ImuManager not initialized, skipping IMU task creation");
        return;
    }
    
    BaseType_t ret = xTaskCreate(ImuDataTask, "imu_data_task", 4096, this, 5, &imu_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU data task");
    } else {
        ESP_LOGI(TAG, "IMU data task created successfully");
    }
}

void ImuManager::StopImuTask() {
    if (!imu_task_handle_) {
        return;
    }
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    if (imu_task_handle_ == self) {
        // 自删分支
        imu_task_handle_ = nullptr;
        vTaskDelete(NULL);
    } else {
        vTaskDelete(imu_task_handle_);
        imu_task_handle_ = nullptr;
    }
}

void ImuManager::ImuDataTask(void *pvParameters) {
    ImuManager *manager = static_cast<ImuManager *>(pvParameters);
    ESP_LOGI(TAG, "IMU data task started");

    complimentary_angle_t angle; // 官方结构仅包含 roll/pitch
    
    // 上一次的角度值用于检测变化
    static complimentary_angle_t last_angle = {0, 0};
    static bool first_reading = true;
    static int error_count = 0;  // 移到这里，避免重复定义
    
    // 睡眠检测专用阈值 - 以姿态角为主
    const float ANGLE_CHANGE_THRESHOLD = ImuDetectConfig::kAngleChangeThresholdDeg;
    
    // 分钟窗口统计（按分钟评估静止比例）
    static uint32_t total_samples = 0;               // 总采样次数（仅用于日志）
    static uint32_t minute_start_time_ms = 0;        // 当前分钟起始时间
    static uint32_t minute_total_samples = 0;        // 当前分钟采样数
    static uint32_t minute_static_samples = 0;       // 当前分钟静止样本数
    static uint32_t consecutive_static_minutes = 0;   // 连续静止分钟数
    
    // 校准占位（当前基于姿态角，不依赖陀螺偏置）
    static bool calibration_done = false;
    static uint32_t calibration_samples = 0;

    while (true) {
        if (manager->mpu6050_sensor_ && manager->initialized_) {
            bool data_valid = true;
            
            // 读取姿态角（内部完成子传感器读取与融合，等价官方互补滤波）
            data_valid &= manager->mpu6050_sensor_->GetAngle(&angle);
            
            if (data_valid) {
                total_samples++;
                
                // 简化校准流程（仅作为启动稳定期计数）
                if (!calibration_done) {
                    if (calibration_samples < 100) {
                        // 姿态角主导，无需累积零偏
                        calibration_samples++;
                        
                        ESP_LOGI(TAG, "Calibrating gyro... %d/100", calibration_samples);
                        
                        // 重置错误计数
                        error_count = 0;
                        last_angle = angle;
                        first_reading = false;
                        continue;
                    } else {
                        calibration_done = true;
                        ESP_LOGI(TAG, "Gyro calibration placeholder done");
                    }
                }
                
                // 姿态角变化检测 - 已经融合了陀螺仪、加速度、温度补偿的最终结果
                float angle_change_magnitude = sqrt(
                    pow(angle.pitch - last_angle.pitch, 2) + 
                    pow(angle.roll - last_angle.roll, 2)
                );
                
                // 以当前阈值定义“静止样本”
                bool is_static_sample = (angle_change_magnitude <= ANGLE_CHANGE_THRESHOLD) && !first_reading;
                
                // 初始化分钟窗口
                uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (minute_start_time_ms == 0) {
                    minute_start_time_ms = now_ms;
                }
                
                // 累计当前分钟统计
                minute_total_samples++;
                if (is_static_sample) {
                    minute_static_samples++;
                }
                
                // 到达一分钟边界 → 评估静止比例
                if (now_ms - minute_start_time_ms >= ImuDetectConfig::kMinuteMs || minute_total_samples >= (ImuDetectConfig::kMinuteMs / ImuDetectConfig::kTaskIntervalMs)) {
                    float minute_static_ratio = minute_total_samples > 0 ? (float)minute_static_samples / (float)minute_total_samples : 0.0f;
                    bool minute_is_static = (minute_static_ratio >= ImuDetectConfig::kMinuteStaticRatio);
                    if (minute_is_static) {
                        consecutive_static_minutes++;
                    } else {
                        consecutive_static_minutes = 0;
                    }
                    ESP_LOGI(TAG, "IMU minute: samples=%u static=%u ratio=%.1f%% consecutive_static_minutes=%u",
                             minute_total_samples, minute_static_samples, minute_static_ratio * 100.0f, consecutive_static_minutes);
                    
                    // 重置下一分钟窗口
                    minute_start_time_ms = now_ms;
                    minute_total_samples = 0;
                    minute_static_samples = 0;
                }
                
                // 连续5分钟静止 → SLEEP
                if (consecutive_static_minutes >= ImuDetectConfig::kConsecutiveStaticMinutesForSleep) {
                    ESP_LOGI(TAG, "IMU: Sleep detected by %u static minutes -> Stop sleep music and IMU task",
                             (unsigned)ImuDetectConfig::kConsecutiveStaticMinutesForSleep);
                    SleepMusicProtocol::GetInstance().StopSleepMusic();
                    auto led = Board::GetInstance().GetLed();
                    led->OnStateChanged();
                    // 任务自删，避免与外部重复删除冲突
                    manager->imu_task_handle_ = nullptr;
                    vTaskDelete(NULL);
                }
                
                // 状态日志（每样本）
                ESP_LOGI(TAG, "IMU: dAngle=%.2f° P=%.1f° R=%.1f° minute[%u/%u static=%u cons=%u]",
                         angle_change_magnitude, angle.pitch, angle.roll,
                         (unsigned)((now_ms - minute_start_time_ms) / 1000),
                         (unsigned)(minute_total_samples),
                         (unsigned)(minute_static_samples),
                         (unsigned)(consecutive_static_minutes));
                
                // 更新上一次的值
                last_angle = angle;
                first_reading = false;
                
                // 重置错误计数
                error_count = 0;
            } else {
                // I2C错误处理和恢复
                error_count++;
                
                ESP_LOGW(TAG, "IMU: Failed to read sensor data (error count: %d)", error_count);
                
                // 连续错误超过5次，尝试重新初始化IMU
                if (error_count >= 5) {
                    ESP_LOGW(TAG, "IMU: Too many errors, attempting to reinitialize...");
                    
                    // 重新初始化IMU传感器
                    if (manager->mpu6050_sensor_) {
                        if (manager->mpu6050_sensor_->WakeUp()) {
                            ESP_LOGI(TAG, "IMU: Sensor reinitialized successfully");
                            error_count = 0;
                        } else {
                            ESP_LOGE(TAG, "IMU: Sensor reinitialization failed");
                        }
                    }
                    
                    // 如果重新初始化失败，延长等待时间
                    if (error_count >= 5) {
                        ESP_LOGW(TAG, "IMU: Entering recovery mode, waiting 2 seconds...");
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        error_count = 0; // 重置计数，重新尝试
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(ImuDetectConfig::kTaskIntervalMs));
    }
}
