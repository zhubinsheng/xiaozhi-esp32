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
    size_t GetPressureSampleCount() const;  // 获取累计采样数
    
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
    void ProcessSample();                          // 处理单次采样（阈值去抖逻辑）
    // 旧的分钟统计逻辑已废弃
    static void AdcTask(void *pvParameters);
    
    bool initialized_ = false;
    adc_oneshot_unit_handle_t adc1_handle_;
    adc_cali_handle_t adc1_cali_handle_;
    
    // 当前检测状态
    DetectionState detection_state_ = kStateWakeUp;
    
    // 压感传感器数据
    int current_pressure_value_ = 0;
    int last_pressure_value_ = 0;
    
    // 新：阈值+去抖配置
    struct AdcDetectConfig {
        static constexpr int kLyingThreshold = 150;           // 中心阈值（兼容保留）
        static constexpr int kLyingHigh = 160;                // 滞回上阈：进入躺下判定
        static constexpr int kLyingLow  = 140;                 // 滞回下阈：起身判定
        static constexpr int kDebounceMs = 5000;              // 去抖时长（毫秒）
        static constexpr int kRefractoryMs = 2000;            // 状态切换保护期（毫秒）
        static constexpr int kDecayAbove = 1;                 // 计数回退步长（above）
        static constexpr int kDecayBelow = 1;                 // 计数回退步长（below）
        static constexpr int64_t kSamplingInterval = 100;     // 统一100ms采样间隔
        static constexpr int kDebounceSamples() { return (int)(kDebounceMs / kSamplingInterval); }
    };
    
    // 去抖计数器（基于采样次数）
    int consecutive_above_threshold_samples_ = 0;      // 连续高于阈值的样本数
    int consecutive_below_threshold_samples_ = 0;      // 连续低于阈值的样本数
    size_t cumulative_samples_ = 0;                    // 累计样本数
    int64_t last_state_change_ms_ = 0;                 // 上次状态切换时间（ms）
    int ema_value_ = 0;                                // EMA平滑值（仅日志展示）
    
    // 助眠模式状态（ADC管理层面）
    bool sleep_mode_active_ = false;                   // 是否已进入助眠模式
    
    // 旧分钟统计缓存与时间控制已移除

    // 任务句柄
    TaskHandle_t adc_task_handle_ = nullptr;
};

#endif // ADC_MANAGER_H
