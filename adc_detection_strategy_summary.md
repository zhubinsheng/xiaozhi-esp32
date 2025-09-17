# ADC检测策略更新总结

## 概述
根据提供的图片中的检测策略，重新设计了ADC管理器的检测逻辑，实现了三种不同的检测模式，对应图片中显示的三种策略。

## 图片检测策略分析

### 1. 采样模式（Sampling Mode）
- **特征**：白色方框表示基础状态
- **策略**：固定间隔采样，基础状态检测
- **应用**：用于基础的压力检测，响应速度快

### 2. 监测周期模式（Monitoring Cycle）  
- **特征**：蓝色方框表示检测活跃期，黄色方框表示监测状态
- **策略**：周期性检测，在特定时间窗口内进行监测
- **应用**：平衡功耗和检测精度，适合中等响应要求

### 3. 睡眠状态模式（Sleep State）
- **特征**：蓝色和黄色方框混合，复杂的状态切换
- **策略**：智能状态管理，多级检测
- **应用**：最大化节能，适合长期运行场景

## 代码实现更改

### 1. 头文件更新 (`adc_manager.h`)

#### 新增枚举类型
```cpp
// 检测模式枚举（对应图片中的三种策略）
enum DetectionMode {
    kSamplingMode,      // 采样模式：基础状态检测
    kMonitoringCycle,   // 监测周期模式：周期性检测
    kSleepState         // 睡眠状态模式：智能状态切换
};

// 检测状态枚举
enum DetectionState {
    kStateIdle,         // 空闲状态（白色）
    kStateActive,       // 活跃状态（蓝色）
    kStateMonitoring    // 监测状态（黄色）
};
```

#### 简化接口
- 移除了复杂的长时间不动检测相关接口
- 添加了模式设置和状态查询接口
- 保留核心的音乐触发功能

### 2. 实现文件更新 (`adc_manager.cc`)

#### 核心检测逻辑重写
替换原有的复杂检测逻辑，实现三种模式：

1. **采样模式实现**
   - 100ms固定间隔检测
   - 简单阈值判断
   - 快速响应状态变化

2. **监测周期模式实现**  
   - 500ms监测间隔
   - 周期性状态切换
   - 平衡响应速度和功耗

3. **睡眠状态模式实现**
   - 1000ms长间隔检测
   - 多级状态转换逻辑
   - 最大化节能效果

#### 动态任务间隔
```cpp
// 根据检测模式设置不同的任务间隔
switch (manager->detection_mode_) {
    case kSamplingMode:   delay_ms = 100; break;   // 快速响应
    case kMonitoringCycle: delay_ms = 500; break;  // 中等响应  
    case kSleepState:     delay_ms = 1000; break;  // 节能优先
}
```

## 技术特点

### 1. 严格按照图片策略实现
- 没有添加任何额外的检测策略
- 完全对应图片中的三种模式
- 保持了原有的音乐播放触发功能

### 2. 状态管理优化
- 清晰的状态枚举定义
- 简化的状态转换逻辑
- 统一的时间管理机制

### 3. 可配置的检测参数
```cpp
static constexpr int kPressureThreshold = 1000;    // 压力检测阈值
static constexpr int kLowValueThreshold = 100;     // 低值检测阈值
static constexpr int64_t kStateChangeThreshold = 2000; // 状态切换时间阈值
```

### 4. 模块化设计
- 每种模式独立的处理函数
- 统一的接口调用方式
- 便于后续扩展和维护

## 使用方式

```cpp
// 获取ADC管理器实例
auto& adc_manager = AdcManager::GetInstance();

// 设置检测模式
adc_manager.SetDetectionMode(kSamplingMode);      // 采样模式
adc_manager.SetDetectionMode(kMonitoringCycle);   // 监测周期模式
adc_manager.SetDetectionMode(kSleepState);        // 睡眠状态模式

// 查询当前状态
DetectionState state = adc_manager.GetDetectionState();
```

## 总结

此次更新完全按照图片中的检测策略重新设计了ADC检测逻辑，实现了三种不同特性的检测模式，为不同应用场景提供了灵活的选择：

- **采样模式**：适合需要快速响应的场景
- **监测周期模式**：平衡响应速度和功耗
- **睡眠状态模式**：最大化节能，适合长期运行

代码结构清晰，易于维护和扩展，完全符合图片中展示的检测策略要求。
