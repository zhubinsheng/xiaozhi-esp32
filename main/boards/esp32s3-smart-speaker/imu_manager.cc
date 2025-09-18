#include "imu_manager.h"
#include <esp_log.h>
#include <cmath>     // 添加数学库，用于fabs、sqrt、pow函数

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
    
    // 启动IMU任务
    StartImuTask();
    
    initialized_ = true;
    ESP_LOGI(TAG, "ImuManager initialized successfully");
    return true;
}

void ImuManager::InitializeImu() {
    ESP_LOGI(TAG, "Initializing MPU6050 IMU sensor...");

    // IMU传感器I2C总线
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

    // 初始化MPU6050传感器
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
                    ESP_LOGE(TAG, "Unsupported IMU device ID: 0x%02X (expected 0x%02X)",
                             device_id, MPU6500_WHO_AM_I_VAL);
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
    if (imu_task_handle_) {
        vTaskDelete(imu_task_handle_);
        imu_task_handle_ = nullptr;
    }
}

void ImuManager::ImuDataTask(void *pvParameters) {
    ImuManager *manager = static_cast<ImuManager *>(pvParameters);
    ESP_LOGI(TAG, "IMU data task started");

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    complimentary_angle_t angle;
    
    // 上一次的角度值和加速度值，用于检测变化
    static complimentary_angle_t last_angle = {0, 0, 0};
    static mpu6050_acce_value_t last_acce = {0, 0, 0};
    static bool first_reading = true;
    static int error_count = 0;  // 移到这里，避免重复定义
    
    // 睡眠检测专用阈值 - 以姿态角为主
    const float ANGLE_CHANGE_THRESHOLD = 2.0f;      // 姿态角变化阈值（度）- 主要检测器
    const float GYRO_MOVEMENT_THRESHOLD = 0.5f;     // 陀螺仪运动阈值（°/s）- 辅助检测器  
    const float ACCE_MOVEMENT_THRESHOLD = 0.03f;    // 加速度微动阈值（g）- 辅助检测器
    
    // 睡眠检测计时器
    static uint32_t movement_count = 0;              // 运动次数计数
    static uint32_t total_samples = 0;               // 总采样次数
    static uint32_t sleep_start_time = 0;            // 开始睡眠检测时间
    
    // 陀螺仪零点校准
    static mpu6050_gyro_value_t gyro_offset = {0, 0, 0};  // 零点偏移
    static bool calibration_done = false;            // 校准完成标志
    static uint32_t calibration_samples = 0;        // 校准采样数

    while (true) {
        if (manager->mpu6050_sensor_ && manager->initialized_) {
            bool data_valid = true;
            
            // 读取所有传感器数据
            data_valid &= manager->mpu6050_sensor_->GetAccelerometer(&acce);
            data_valid &= manager->mpu6050_sensor_->GetGyroscope(&gyro);
            data_valid &= manager->mpu6050_sensor_->GetTemperature(&temp);
            data_valid &= manager->mpu6050_sensor_->ComplimentaryFilter(&acce, &gyro, &angle);
            
            if (data_valid) {
                total_samples++;
                
                // 陀螺仪零点校准（前100个样本）
                if (!calibration_done) {
                    if (calibration_samples < 100) {
                        gyro_offset.gyro_x += gyro.gyro_x;
                        gyro_offset.gyro_y += gyro.gyro_y;
                        gyro_offset.gyro_z += gyro.gyro_z;
                        calibration_samples++;
                        
                        ESP_LOGI(TAG, "Calibrating gyro... %d/100", calibration_samples);
                        
                        // 重置错误计数
                        error_count = 0;
                        last_angle = angle;
                        last_acce = acce;
                        first_reading = false;
                        continue;
                    } else {
                        // 计算平均偏移值
                        gyro_offset.gyro_x /= 100;
                        gyro_offset.gyro_y /= 100;
                        gyro_offset.gyro_z /= 100;
                        calibration_done = true;
                        
                        ESP_LOGI(TAG, "Gyro calibration done: offset(%.2f, %.2f, %.2f)°/s", 
                                gyro_offset.gyro_x, gyro_offset.gyro_y, gyro_offset.gyro_z);
                    }
                }
                
                // 应用零点校准
                mpu6050_gyro_value_t calibrated_gyro;
                calibrated_gyro.gyro_x = gyro.gyro_x - gyro_offset.gyro_x;
                calibrated_gyro.gyro_y = gyro.gyro_y - gyro_offset.gyro_y;
                calibrated_gyro.gyro_z = gyro.gyro_z - gyro_offset.gyro_z;
                
                // 睡眠检测算法：纯姿态角检测（已融合所有传感器数据，最准确）
                
                // 姿态角变化检测 - 已经融合了陀螺仪、加速度、温度补偿的最终结果
                float angle_change_magnitude = sqrt(
                    pow(angle.pitch - last_angle.pitch, 2) + 
                    pow(angle.roll - last_angle.roll, 2) + 
                    pow(angle.yaw - last_angle.yaw, 2)
                );
                
                // 纯姿态角判断：最稳定、最准确的方法
                bool movement_detected = first_reading || (angle_change_magnitude > ANGLE_CHANGE_THRESHOLD);
                
                if (movement_detected) {
                    movement_count++;
                    sleep_start_time = 0;  // 重置睡眠计时
                }
                
                // 计算运动比例和睡眠状态
                float movement_ratio = (float)movement_count / total_samples;
                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                
                // 睡眠判断逻辑
                const char* sleep_status = "";
                if (total_samples < 50) {  // 前5秒初始化期
                    sleep_status = "[INITIALIZING]";
                } else if (movement_ratio > 0.1f) {  // 10%以上时间有运动
                    sleep_status = "[AWAKE]";
                    sleep_start_time = 0;
                } else {
                    if (sleep_start_time == 0) {
                        sleep_start_time = current_time;
                        sleep_status = "[GETTING_SLEEPY]";
                    } else {
                        uint32_t sleep_duration = current_time - sleep_start_time;
                        if (sleep_duration > 20000) {  // 20秒无显著运动
                            sleep_status = "[SLEEPING]";
                        } else {
                            sleep_status = "[DROWSY]";
                        }
                    }
                }
                
                // 显示简化状态（重点显示姿态角变化）
                ESP_LOGI(TAG, "Sleep: Angle:%.2f°(P:%.1f° R:%.1f° Y:%.1f°) Move:%d/%d(%.1f%%) %s",
                    angle_change_magnitude,
                    angle.pitch, angle.roll, angle.yaw,
                    movement_count, total_samples, movement_ratio * 100,
                    sleep_status);
                
                // 更新上一次的值
                last_angle = angle;
                last_acce = acce;
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
        vTaskDelay(pdMS_TO_TICKS(160)); // 改为50ms间隔，减少日志输出频率
    }
}
