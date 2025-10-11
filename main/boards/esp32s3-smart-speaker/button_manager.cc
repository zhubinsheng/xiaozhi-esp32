#include "button_manager.h"
#include "application.h"
#include "board.h"
#include "display/display.h"
#include "protocols/sleep_music_protocol.h"
#include "device_state.h"
#include <cJSON.h>
#include <esp_log.h>

#define TAG "ButtonManager"

ButtonManager &ButtonManager::GetInstance() {
  static ButtonManager instance;
  return instance;
}

ButtonManager::ButtonManager()
    : boot_button_(BOOT_BUTTON_GPIO), volume_up_button_(VOLUME_UP_BUTTON_GPIO),
      volume_down_button_(VOLUME_DOWN_BUTTON_GPIO),
      test_button_(TEST_BUTTON_GPIO) {}

bool ButtonManager::Initialize() {
  if (initialized_) {
    ESP_LOGW(TAG, "ButtonManager already initialized");
    return true;
  }

  ESP_LOGI(TAG, "Initializing ButtonManager...");

  // 设置按钮回调
  SetupButtonCallbacks();

  initialized_ = true;
  ESP_LOGI(TAG, "ButtonManager initialized successfully");
  return true;
}

void ButtonManager::SetupButtonCallbacks() {
  ESP_LOGI(TAG, "Setting up button callbacks...");

  // BOOT按钮回调
  boot_button_.OnClick([]() { ESP_LOGI(TAG, "Boot button clicked"); });

  boot_button_.OnLongPress([]() {
    ESP_LOGI(TAG, "BOOT long pressed: play boot tone");

   
  });

  // 音量上按钮回调
  volume_up_button_.OnClick([]() {
    ESP_LOGI(TAG, "Volume up button clicked");
    // 通过AudioService间接控制音量
    auto &board = Board::GetInstance();
    auto codec = board.GetAudioCodec();
    codec->SetOutputVolume(codec->output_volume() + 10);
    ESP_LOGI(TAG, "Volume up requested");
  });

  volume_up_button_.OnLongPress([]() {
    ESP_LOGI(TAG,
             "Volume up long pressed: switching to voice interaction mode");

    auto &sleep_protocol = SleepMusicProtocol::GetInstance();
    sleep_protocol.CloseAudioChannel();
    
    // 暂停音乐播放
    auto music = Board::GetInstance().GetMusic();
    if (music && music->IsPlaying()) {
      music->PauseSong();
      ESP_LOGI(TAG, "Music paused for voice interaction");
    }

     // 播放进入语音交互模式的提示音
     auto &app = Application::GetInstance();
     auto device_state = app.GetDeviceState();
     ESP_LOGI(TAG, "Device state: %d", device_state);
     if (device_state == kDeviceStateIdle) {
       app.PlaySound("success"); // 播放成功提示音
       app.ToggleChatState();
     }
  });

  // 音量下按钮回调
  volume_down_button_.OnClick([]() {
    ESP_LOGI(TAG, "Volume down button clicked");
    auto &board = Board::GetInstance();
    auto codec = board.GetAudioCodec();
    codec->SetOutputVolume(codec->output_volume() - 10);
    ESP_LOGI(TAG, "Volume down requested");
  });

  volume_down_button_.OnLongPress([]() {
    ESP_LOGI(TAG, "Volume down long pressed");

    auto &app = Application::GetInstance();
    auto device_state = app.GetDeviceState();
    auto &sleep_protocol = SleepMusicProtocol::GetInstance();
    ESP_LOGI(TAG, "Device state: %d", device_state);

    // 优先级1: 如果在对话中，则关闭对话
    if (device_state > kDeviceStateIdle && device_state < kDeviceStateUpgrading) {
      ESP_LOGI(TAG, "In conversation - stopping voice interaction");
      app.PlaySound("exclamation"); // 播放停止提示音
      // 1) 打断TTS/回复
      app.AbortSpeaking(kAbortReasonNone);
      // 2) 通知上游停止监听
      if (auto* proto = app.GetProtocol()) {
          proto->SendStopListening();
          if (proto->IsAudioChannelOpened()) {
              proto->CloseAudioChannel();
          }
      }
      app.SetDeviceState(kDeviceStateIdle);

      return;
    }
    
    // 优先级2: 如果在助眠模式中，则关闭助眠模式
    if (sleep_protocol.IsAudioChannelOpened()) {
      ESP_LOGI(TAG, "In sleep mode - stopping sleep music");
      app.PlaySound("exclamation"); // 播放停止提示音
      sleep_protocol.StopSleepMusic();
      auto led = Board::GetInstance().GetLed();
      led->OnStateChanged();
      return;
    }
    
    // 优先级3: 如果都不在，则打开助眠模式
    ESP_LOGI(TAG, "Idle state - starting sleep mode");
    app.PlaySound("success"); // 播放成功提示音
    if (sleep_protocol.OpenAudioChannel()) {
      ESP_LOGI(TAG, "Sleep music started successfully");
    } else {
      ESP_LOGI(TAG, "Failed to start sleep music");
    }
  });

  test_button_.OnLongPress([]() {
    ESP_LOGI(TAG, "Test button clicked - sending text message to server");
    
  });

  test_button_.OnClick([]() {
    ESP_LOGI(TAG, "Test button long pressed - simulating wake word detection");
    
  });
}
