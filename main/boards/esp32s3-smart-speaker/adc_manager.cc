#include "adc_manager.h"
#include "application.h"
#include <board.h>
#include <cmath>
#include <cstring>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "protocols/sleep_music_protocol.h"
#include "protocol.h"
#include "config.h"

#define TAG "AdcManager"

AdcManager &AdcManager::GetInstance() {
  static AdcManager instance;
  return instance;
}

bool AdcManager::Initialize() {
  if (initialized_) {
    ESP_LOGW(TAG, "AdcManager already initialized");
    return true;
  }

  ESP_LOGI(TAG, "Initializing AdcManager...");

  InitializeAdc();

  // 先设置初始化状态，再启动任务
  initialized_ = true;

  // 初始化后立刻读取一次，便于快速确认链路
  int init_read_raw = -1;
  esp_err_t init_read_ret = adc_oneshot_read(
      adc1_handle_, PRESSURE_SENSOR_ADC_LEFT_CHANNEL, &init_read_raw);
  if (init_read_ret != ESP_OK) {
    ESP_LOGE(TAG, "Initial ADC read failed: %s",
             esp_err_to_name(init_read_ret));
  } else {
    ESP_LOGI(TAG, "Initial ADC read ok: Raw=%d", init_read_raw);
    // 初始化成功设置为起床状态
    detection_state_ = kStateWakeUp;
  }

  // 启动ADC任务
  StartAdcTask();

  ESP_LOGI(TAG, "AdcManager initialized successfully");
  return true;
}

void AdcManager::InitializeAdc() {
  ESP_LOGI(TAG, "Initializing ADC for pressure sensor on GPIO4 (ADC1_CH3)...");

  // 初始化ADC驱动（完整配置）
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .clk_src = ADC_RTC_CLK_SRC_RC_FAST,  // 使用更稳定的RC快速时钟源
      .ulp_mode = ADC_ULP_MODE_DISABLE,    // 禁用ULP模式
  };
  ESP_LOGI(TAG, "ADC unit config - clk_src: RC_FAST, ulp_mode: DISABLED");
  esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "ADC unit initialized successfully");

  // 配置ADC通道 (优化衰减设置)
  adc_oneshot_chan_cfg_t chan_config = {
      .atten = ADC_ATTEN_DB_6,    // 改为6dB衰减(0-2.2V)，提高精度
      .bitwidth = ADC_BITWIDTH_12,
  };
  ret = adc_oneshot_config_channel(adc1_handle_, PRESSURE_SENSOR_ADC_LEFT_CHANNEL,
                                   &chan_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s",
             PRESSURE_SENSOR_ADC_LEFT_CHANNEL, esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "ADC channel %d configured successfully",
           PRESSURE_SENSOR_ADC_LEFT_CHANNEL);

  // 初始化ADC校准 (与通道配置保持一致)
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = PRESSURE_SENSOR_ADC_LEFT_CHANNEL,  // 添加通道参数
      .atten = ADC_ATTEN_DB_6,     // 与通道配置保持一致
      .bitwidth = ADC_BITWIDTH_12,
  };
  ESP_LOGI(TAG, "ADC calibration config - chan: %d, atten: 6dB, bitwidth: 12bit", 
           PRESSURE_SENSOR_ADC_LEFT_CHANNEL);
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "ADC calibration not available: %s", esp_err_to_name(ret));
    adc1_cali_handle_ = NULL;
  } else {
    ESP_LOGI(TAG, "ADC calibration initialized successfully");
  }

  ESP_LOGI(TAG, "ADC initialized for pressure sensor monitoring on GPIO4");
  ESP_LOGI(TAG, "ADC Config - Channel: %d, Attenuation: 6dB, Bitwidth: 12bit, Range: 0-2.2V", 
           PRESSURE_SENSOR_ADC_LEFT_CHANNEL);
}

void AdcManager::ReadPressureSensorData() {
  if (!initialized_) {
    return;
  }

  int adc_value;
  esp_err_t ret =
      adc_oneshot_read(adc1_handle_, PRESSURE_SENSOR_ADC_LEFT_CHANNEL, &adc_value);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read pressure sensor ADC: %s",
             esp_err_to_name(ret));
    return;
  }

  static int log_counter = 0;
  if (++log_counter >= 100) { // 100次*100ms=10秒
    ESP_LOGI(TAG, "ADC value: %d", adc_value);
    log_counter = 0;
  }
  // 更新压力值
  last_pressure_value_ = current_pressure_value_;
  current_pressure_value_ = adc_value;

  // 处理采样数据
  ProcessSample();
}

void AdcManager::StartAdcTask() {
  if (!initialized_) {
    ESP_LOGE(TAG, "AdcManager not initialized");
    return;
  }

  BaseType_t ret =
      xTaskCreate(AdcTask, "adc_task", 4096, this, 2, &adc_task_handle_);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create ADC task");
  } else {
    ESP_LOGI(TAG, "ADC task created successfully");
  }
}

void AdcManager::StopAdcTask() {
  if (adc_task_handle_) {
    vTaskDelete(adc_task_handle_);
    adc_task_handle_ = nullptr;
  }
}

void AdcManager::AdcTask(void *pvParameters) {
  AdcManager *manager = static_cast<AdcManager *>(pvParameters);
  ESP_LOGI(TAG, "ADC task started");

  while (true) {
    if (manager->initialized_) {
      manager->ReadPressureSensorData();
    }
    // 固定100ms采样间隔
    vTaskDelay(pdMS_TO_TICKS(kSamplingInterval));
  }
}

int AdcManager::GetCurrentPressureValue() const {
  return current_pressure_value_;
}

size_t AdcManager::GetPressureSampleCount() const {
  return current_sample_index_;  // 返回当前分钟已采集的样本数
}

void AdcManager::ProcessSample() {
  int64_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
  
  // 初始化时间
  if (minute_start_time_ == 0) {
    minute_start_time_ = current_time;
  }
  
  // 将当前采样存储到缓存中
  minute_samples_[current_sample_index_] = current_pressure_value_;
  
  // 判断是否有运动，更新静止计数
  if (current_sample_index_ > 0) { // 需要有上一次采样作为参考
    if (!IsMovement(current_pressure_value_, minute_samples_[current_sample_index_ - 1])) {
      static_count_in_minute_++;
    }
  }
  
  current_sample_index_++;
  
  // 检查是否完成一分钟的采样
  int samples_per_minute = GetSamplesPerMinute();
  if (current_sample_index_ >= samples_per_minute || 
      (current_time - minute_start_time_) >= 60000) {
    AnalyzeMinuteData();
    
    // 重置一分钟的数据
    current_sample_index_ = 0;
    static_count_in_minute_ = 0;
    minute_start_time_ = current_time;
    memset(minute_samples_, 0, sizeof(minute_samples_));
    
    ESP_LOGI(TAG, "Minute reset: samples_per_minute=%d", GetSamplesPerMinute());
  }
}

void AdcManager::AnalyzeMinuteData() {
  // 计算静止比例
  float static_ratio = (float)static_count_in_minute_ / (float)current_sample_index_;
  
  ESP_LOGI(TAG, "Minute analysis: samples=%d, static=%d, ratio=%.2f", 
           current_sample_index_, static_count_in_minute_, static_ratio);
  
  if (static_ratio >= kStaticRatio) {
    // 这一分钟是静止的
    consecutive_static_minutes_++;
    ESP_LOGI(TAG, "Static minute detected, consecutive: %d", consecutive_static_minutes_);
    
    // 更新状态为躺下状态
    if (detection_state_ != kStateLyingDown) {
      detection_state_ = kStateLyingDown;
      // 通知LED更新状态
      auto led = Board::GetInstance().GetLed();
      led->OnStateChanged();
      ESP_LOGI(TAG, "State changed to LyingDown due to static behavior");
    }
  } else {
    // 这一分钟有活动，重置计数
    consecutive_static_minutes_ = 0;
    ESP_LOGI(TAG, "Active minute detected, reset sleep counter");
    
    // 更新状态为起床状态
    if (detection_state_ != kStateWakeUp) {
      detection_state_ = kStateWakeUp;
      // 通知LED更新状态
      auto led = Board::GetInstance().GetLed();
      led->OnStateChanged();
      ESP_LOGI(TAG, "State changed to WakeUp due to movement");
      TriggerMusicPlayback();
    }
  }
  
  // 检查是否达到睡眠条件
  UpdateSleepState();
}

bool AdcManager::IsMovement(int current_val, int last_val) const {
  int diff = abs(current_val - last_val);
  return diff > kMovementThreshold;
}

void AdcManager::UpdateSleepState() {
  if (consecutive_static_minutes_ >= kSleepMinutes) {
    if (detection_state_ != kStateSleeping) {
      detection_state_ = kStateSleeping;
      // 通知LED更新状态
      auto led = Board::GetInstance().GetLed();
      led->OnStateChanged();
      ESP_LOGI(TAG, "Sleep detected! %d consecutive static minutes", consecutive_static_minutes_);
      TriggerMusicPauseback();
    }
  }
}

void AdcManager::TriggerMusicPauseback() {
  ESP_LOGI(TAG, "Triggering sleep music pauseback");
  auto& sleep_protocol = SleepMusicProtocol::GetInstance();
  // sleep_protocol.StopSleepMusic();
}

void AdcManager::TriggerMusicPlayback() {
  ESP_LOGI(TAG, "Triggering sleep music playback");
  auto& sleep_protocol = SleepMusicProtocol::GetInstance();
  // sleep_protocol.StartSleepMusic();
}
