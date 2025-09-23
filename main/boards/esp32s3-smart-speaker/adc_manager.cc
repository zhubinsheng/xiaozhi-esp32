#include "adc_manager.h"
#include "application.h"
#include <board.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "protocols/sleep_music_protocol.h"
#include "config.h"
#include "imu_manager.h"

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

  // 确保模拟引脚不被数字GPIO干扰：禁用方向与上下拉
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_DISABLE;
  io_conf.pin_bit_mask = (1ULL << GPIO_NUM_4);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // 初始化ADC驱动（完整配置）
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      // .clk_src = ADC_RTC_CLK_SRC_RC_FAST,  // 使用更稳定的RC快速时钟源
      // .ulp_mode = ADC_ULP_MODE_DISABLE,    // 禁用ULP模式
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
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
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
      .atten = ADC_ATTEN_DB_12,     // 与通道配置保持一致
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_LOGI(TAG, "ADC calibration config - chan: %d, atten: 12dB, bitwidth: 12bit", 
           PRESSURE_SENSOR_ADC_LEFT_CHANNEL);
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "ADC calibration not available: %s", esp_err_to_name(ret));
    adc1_cali_handle_ = NULL;
  } else {
    ESP_LOGI(TAG, "ADC calibration initialized successfully");
  }

  ESP_LOGI(TAG, "ADC initialized for pressure sensor monitoring");
  ESP_LOGI(TAG, "ADC Config - Channel: %d, Attenuation: 12dB, Bitwidth: 12bit, Range: ~0-3.3V", 
           PRESSURE_SENSOR_ADC_LEFT_CHANNEL);
}

void AdcManager::ReadPressureSensorData() {
  if (!initialized_) {
    return;
  }

  // 多次采样 + 去极值平均，降低尖峰/串扰影响
  static constexpr int kReads = 12;
  int values[kReads];
  int success_count = 0;
  for (int i = 0; i < kReads; ++i) {
    int v = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle_, PRESSURE_SENSOR_ADC_LEFT_CHANNEL, &v);
    if (ret == ESP_OK) {
      values[success_count++] = v;
    }
  }
  if (success_count == 0) {
    ESP_LOGE(TAG, "Failed to read pressure sensor ADC: all samples failed");
    return;
  }
  // 单遍扫描求和+极值，避免排序带来的小数组静态阈值告警
  int min1 = 0x7FFFFFFF, min2 = 0x7FFFFFFF;
  int max1 = -0x7FFFFFFF, max2 = -0x7FFFFFFF;
  long sum = 0;
  for (int i = 0; i < success_count; ++i) {
    int v = values[i];
    sum += v;
    if (v < min1) { min2 = min1; min1 = v; }
    else if (v < min2) { min2 = v; }
    if (v > max1) { max2 = max1; max1 = v; }
    else if (v > max2) { max2 = v; }
  }
  int remove_cnt = 0;
  long remove_sum = 0;
  if (success_count >= 8) { // 丢2大2小
    remove_cnt = 4;
    remove_sum = (long)min1 + min2 + max1 + max2;
  } else if (success_count >= 5) { // 丢1大1小
    remove_cnt = 2;
    remove_sum = (long)min1 + max1;
  }
  int denom = success_count - remove_cnt;
  int adc_value = (denom > 0) ? static_cast<int>((sum - remove_sum) / denom) : values[0];

  static int log_counter = 0;
  if (++log_counter >= 10) { // 10次*100ms=1秒
    // EMA 仅用于日志观测趋势，不参与判定
    if (cumulative_samples_ == 0) {
      ema_value_ = adc_value;
    } else {
      // alpha=0.2 -> 80%保留历史
      ema_value_ = (ema_value_ * 8 + adc_value * 2) / 10;
    }
    ESP_LOGI(TAG, "ADC value: %d (ema:%d)", adc_value, ema_value_);
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
    // 固定采样间隔
    vTaskDelay(pdMS_TO_TICKS(AdcDetectConfig::kSamplingInterval));
  }
}

int AdcManager::GetCurrentPressureValue() const {
  return current_pressure_value_;
}

size_t AdcManager::GetPressureSampleCount() const {
  return cumulative_samples_;  // 返回累计样本数
}

void AdcManager::ProcessSample() {
  // 基于滞回阈值+去抖+保护期：>=kLyingHigh 持续5秒判定躺下；<kLyingLow 持续5秒判定起身
  const int debounce_samples = AdcDetectConfig::kDebounceSamples();
  const int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000);

  // 仅在“待进入躺下”阶段统计 above；仅在“已躺下”阶段统计 below（可配置衰减步长）
  if (detection_state_ != kStateLyingDown) {
    // 正在等待判定躺下：只统计 >= 上阈 的连续计数；< 下阈 时对 above 衰减
    if (current_pressure_value_ >= AdcDetectConfig::kLyingHigh) {
      consecutive_above_threshold_samples_++;
    } else {
      if (consecutive_above_threshold_samples_ > 0) {
        consecutive_above_threshold_samples_ = std::max(0, consecutive_above_threshold_samples_ - AdcDetectConfig::kDecayAbove);
      }
    }
    // 非躺下阶段，不统计 below
    consecutive_below_threshold_samples_ = 0;
  } else {
    // 已处于躺下：只统计 < 下阈 的连续计数；>= 上阈 时对 below 衰减
    if (current_pressure_value_ < AdcDetectConfig::kLyingLow) {
      consecutive_below_threshold_samples_++;
    } else {
      if (consecutive_below_threshold_samples_ > 0) {
        consecutive_below_threshold_samples_ = std::max(0, consecutive_below_threshold_samples_ - AdcDetectConfig::kDecayBelow);
      }
    }
    // 躺下阶段，不再关注 above
    consecutive_above_threshold_samples_ = 0;
  }

  // 累计样本
  cumulative_samples_++;
  ESP_LOGI(TAG, "ADC value: %d (ema:%d), above:%d, below:%d, total:%d",
          current_pressure_value_, ema_value_, consecutive_above_threshold_samples_, consecutive_below_threshold_samples_, cumulative_samples_);

  // 进入躺下
  if (consecutive_above_threshold_samples_ >= debounce_samples && detection_state_ != kStateLyingDown) {
    // 保护期：刚切换后的一小段时间不允许反向切换
    if (last_state_change_ms_ != 0 && (now_ms - last_state_change_ms_) < AdcDetectConfig::kRefractoryMs) {
      return;
    }
    detection_state_ = kStateLyingDown;
    last_state_change_ms_ = now_ms;
    auto& app = Application::GetInstance();
    auto device_state = app.GetDeviceState();
    bool in_conversation = (device_state == kDeviceStateListening || device_state == kDeviceStateSpeaking);
    ESP_LOGI(TAG, "ADC lying down detected: value=%d, above_cnt=%d(~%d ms), in_conversation=%s, sleep_mode_active(before)=%s",
             current_pressure_value_, consecutive_above_threshold_samples_, AdcDetectConfig::kDebounceMs,
             in_conversation ? "true" : "false",
             sleep_mode_active_ ? "true" : "false");

    // 若不在对话中，则进入助眠模式并启动IMU
    if (!in_conversation && !sleep_mode_active_) {
      TriggerMusicPlayback();
      ImuManager::GetInstance().StartImuTask();
      sleep_mode_active_ = true;
      ESP_LOGI(TAG, "Sleep mode entered: sleep_mode_active(now)=%s",
               sleep_mode_active_ ? "true" : "false");
    }
  }

  // 离开躺下（起身）
  if (consecutive_below_threshold_samples_ >= debounce_samples && detection_state_ != kStateWakeUp) {
    if (last_state_change_ms_ != 0 && (now_ms - last_state_change_ms_) < AdcDetectConfig::kRefractoryMs) {
      return;
    }
    detection_state_ = kStateWakeUp;
    last_state_change_ms_ = now_ms;
    auto& app = Application::GetInstance();
    auto device_state = app.GetDeviceState();
    bool in_conversation = (device_state == kDeviceStateListening || device_state == kDeviceStateSpeaking);
    ESP_LOGI(TAG, "ADC wake up detected: value=%d, below_cnt=%d(~%d ms), in_conversation=%s, sleep_mode_active(before)=%s",
             current_pressure_value_, consecutive_below_threshold_samples_, AdcDetectConfig::kDebounceMs,
             in_conversation ? "true" : "false",
             sleep_mode_active_ ? "true" : "false");

    // 停止助眠并关闭IMU
    if (sleep_mode_active_) {
      TriggerMusicPauseback();
      ImuManager::GetInstance().StopImuTask();
      sleep_mode_active_ = false;
      ESP_LOGI(TAG, "Sleep mode exited: sleep_mode_active(now)=%s",
               sleep_mode_active_ ? "true" : "false");
    }
  }
}

void AdcManager::TriggerMusicPauseback() {
  ESP_LOGI(TAG, "Triggering sleep music pauseback");
  auto& sleep_protocol = SleepMusicProtocol::GetInstance();
  sleep_protocol.StopSleepMusic();
  auto led = Board::GetInstance().GetLed();
  led->OnStateChanged();
}

void AdcManager::TriggerMusicPlayback() {
  ESP_LOGI(TAG, "Triggering sleep music playback");
  auto& sleep_protocol = SleepMusicProtocol::GetInstance();
  sleep_protocol.StartSleepMusic();
  auto led = Board::GetInstance().GetLed();
  led->OnStateChanged();
}
