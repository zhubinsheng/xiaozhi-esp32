#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 检测状态枚举（对应LED状态指示）
enum DetectionState {
    kStateWakeUp,       // 起床状态
    kStateLyingDown,    // 躺下状态 - 黄色常亮 (P1)
    kStateSleeping,     // 睡着状态 - 红色常亮 (P1)
};

class AdcManager {
public:
    static AdcManager& GetInstance();
    
    // 初始化ADC系统
    bool Initialize();
    
    // 读取压感传感器数据
    void ReadPressureSensorData();
    
    // 获取当前压感值
    int GetCurrentPressureValue() const;
    
    // 获取压感样本统计信息
    size_t GetPressureSampleCount() const;  // 获取当前分钟已采集的样本数
    
    // 启动/停止ADC任务
    void StartAdcTask();
    void StopAdcTask();
    
    // 检查是否已初始化
    bool IsInitialized() const { return initialized_; }
    
    
    // 获取当前检测状态
    DetectionState GetDetectionState() const { return detection_state_; }
    
    // 压力检测触发音乐播放
    void TriggerMusicPlayback();
    void TriggerMusicPauseback();

private:
    AdcManager() = default;
    ~AdcManager() = default;
    AdcManager(const AdcManager&) = delete;
    AdcManager& operator=(const AdcManager&) = delete;
    
    void InitializeAdc();
    void ProcessSample();                          // 处理单次采样
    void AnalyzeMinuteData();                      // 分析一分钟数据
    bool IsMovement(int current_val, int last_val) const; // 判断是否有运动
    void UpdateSleepState();                       // 更新睡眠状态
    static void AdcTask(void *pvParameters);
    
    bool initialized_ = false;
    adc_oneshot_unit_handle_t adc1_handle_;
    adc_cali_handle_t adc1_cali_handle_;
    
    // 当前检测状态
    DetectionState detection_state_ = kStateWakeUp;
    
    // 压感传感器数据
    int current_pressure_value_ = 0;
    int last_pressure_value_ = 0;
    
    // 统计分析相关变量
    static constexpr int kMovementThreshold = 20;  // 运动检测变化阈值
    static constexpr float kStaticRatio = 0.9f;    // 静止比例阈值 (90%)
    static constexpr int kSleepMinutes = 5;        // 连续静止5分钟判定为睡眠
    
    // 采样间隔配置（毫秒）
    static constexpr int64_t kSamplingInterval = 100;  // 统一100ms采样间隔
    
    // 计算每分钟采样次数
    int GetSamplesPerMinute() const {
        return 60000 / kSamplingInterval;  // 60秒 / 100ms = 600次
    }
    
    // 数据缓存（动态大小，最大按100ms间隔计算）
    static constexpr int kMaxSamplesPerMinute = 600;  // 最大采样数（100ms间隔）
    int minute_samples_[kMaxSamplesPerMinute];        // 当前分钟的采样数据
    int current_sample_index_ = 0;                    // 当前采样索引
    int static_count_in_minute_ = 0;                  // 当前分钟静止次数
    int consecutive_static_minutes_ = 0;              // 连续静止分钟数
    
    // 时间控制
    int64_t last_sample_time_ = 0;
    int64_t minute_start_time_ = 0;
    
    // 任务句柄
    TaskHandle_t adc_task_handle_ = nullptr;
};

#endif // ADC_MANAGER_H
