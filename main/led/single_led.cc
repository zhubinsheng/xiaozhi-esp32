#include "single_led.h"
#include "application.h"
#include "boards/esp32s3-smart-speaker/adc_manager.h"
#include <esp_log.h> 

#define TAG "SingleLed"

#define DEFAULT_BRIGHTNESS 4
#define HIGH_BRIGHTNESS 16
#define LOW_BRIGHTNESS 2

#define BLINK_INFINITE -1


SingleLed::SingleLed(gpio_num_t gpio) {
    // If the gpio is not connected, you should use NoLed class
    assert(gpio != GPIO_NUM_NC);

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = gpio;
    strip_config.max_leds = 1;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_));
    led_strip_clear(led_strip_);

    esp_timer_create_args_t blink_timer_args = {
        .callback = [](void *arg) {
            auto led = static_cast<SingleLed*>(arg);
            led->OnBlinkTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "blink_timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &blink_timer_));
}

SingleLed::~SingleLed() {
    esp_timer_stop(blink_timer_);
    if (led_strip_ != nullptr) {
        led_strip_del(led_strip_);
    }
}


void SingleLed::SetColor(uint8_t r, uint8_t g, uint8_t b) {
    r_ = r;
    g_ = g;
    b_ = b;
}

void SingleLed::TurnOn() {
    if (led_strip_ == nullptr) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
    led_strip_refresh(led_strip_);
}

void SingleLed::TurnOff() {
    if (led_strip_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    led_strip_clear(led_strip_);
}

void SingleLed::BlinkOnce() {
    Blink(1, 100);
}

void SingleLed::Blink(int times, int interval_ms) {
    StartBlinkTask(times, interval_ms);
}

void SingleLed::StartContinuousBlink(int interval_ms) {
    StartBlinkTask(BLINK_INFINITE, interval_ms);
}

void SingleLed::StartBlinkTask(int times, int interval_ms) {
    if (led_strip_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    
    blink_counter_ = times * 2;
    blink_interval_ms_ = interval_ms;
    esp_timer_start_periodic(blink_timer_, interval_ms * 1000);
}

void SingleLed::OnBlinkTimer() {
    std::lock_guard<std::mutex> lock(mutex_);
    blink_counter_--;
    if (blink_counter_ & 1) {
        led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
        led_strip_refresh(led_strip_);
    } else {
        led_strip_clear(led_strip_);

        if (blink_counter_ == 0) {
            esp_timer_stop(blink_timer_);
        }
    }
}


void SingleLed::OnStateChanged() {
    auto& app = Application::GetInstance();
    auto device_state = app.GetDeviceState();
    
    // 优先检查压感检测状态 (P0优先级)
    auto& adc_manager = AdcManager::GetInstance();
    if (adc_manager.IsInitialized()) {
        auto detection_state = adc_manager.GetDetectionState();
        switch (detection_state) {
            case kStateWakeUp:
                // 移除旧的“起床状态关灯”逻辑，由上层状态统一驱动
                break;
                
            case kStateLyingDown:
                SetColor(DEFAULT_BRIGHTNESS, DEFAULT_BRIGHTNESS, 0);  // 黄色常亮 - 躺下状态
                TurnOn();
                ESP_LOGI(TAG, "LED: Yellow (Lying Down State)");
                return;
                
            case kStateSleeping:
                SetColor(DEFAULT_BRIGHTNESS, 0, 0);  // 红色常亮 - 睡着状态
                TurnOn();
                ESP_LOGI(TAG, "LED: Red (Sleeping State)");
                return;
        }
    }
    
    // 如果没有压感状态，则使用系统状态 (P1优先级)
    switch (device_state) {
        case kDeviceStateStarting:
            SetColor(0, 0, DEFAULT_BRIGHTNESS); // 蓝色（Blue）
            StartContinuousBlink(100); // 快速闪烁（Fast blink）
            break;
        case kDeviceStateWifiConfiguring:
            SetColor(0, 0, DEFAULT_BRIGHTNESS); // 蓝色（Blue）
            StartContinuousBlink(500); // 慢速闪烁（Slow blink）
            break;
        case kDeviceStateIdle:
            TurnOff(); // 熄灭（Off）
            break;
        case kDeviceStateConnecting:
            SetColor(0, 0, DEFAULT_BRIGHTNESS); // 蓝色常亮（Blue solid）
            TurnOn();
            break;
        case kDeviceStateListening:
        case kDeviceStateAudioTesting:
            if (app.IsVoiceDetected()) {
                SetColor(HIGH_BRIGHTNESS, 0, 0); // 红色（Red，高亮）
            } else {
                SetColor(LOW_BRIGHTNESS, 0, 0); // 红色（Red，低亮）
            }
            TurnOn(); // 红色常亮（Red solid）
            break;
        case kDeviceStateSpeaking:
            SetColor(0, DEFAULT_BRIGHTNESS, 0); // 绿色（Green）
            TurnOn(); // 绿色常亮（Green solid）
            break;
        case kDeviceStateUpgrading:
            SetColor(0, DEFAULT_BRIGHTNESS, 0); // 绿色（Green）
            StartContinuousBlink(100); // 快速闪烁（Fast blink）
            break;
        case kDeviceStateActivating:
            SetColor(0, DEFAULT_BRIGHTNESS, 0); // 绿色（Green）
            StartContinuousBlink(500); // 慢速闪烁（Slow blink）
            break;
        default:
            ESP_LOGW(TAG, "Unknown led strip event: %d", device_state);
            return;
    }
}
