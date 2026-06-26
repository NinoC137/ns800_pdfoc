# NS800 PD FOC RT-Thread 应用工程说明

本工程是基于 NS800RT7P65 的 RT-Thread 应用层固件。核心应用代码位于：

```text
ns800rt7p65-nssinepad/applications
```

## 应用入口

`applications/main.c` 保持极薄，只负责调用：

```c
return ns800_app_main();
```

实际启动流程位于 `applications/ns800_app.c`。

启动顺序如下：

1. 初始化中断模块和中断向量表。
2. 启动 LED1 子线程。
3. 启动按键扫描线程。
4. 启动 OLED 显示线程。
5. 初始化 EPWM8-EPWM13 功率 PWM 输出。
6. 启动 ADC 后台初始化线程。
7. 启动 EPWM2 功率控制中断线程。
8. 进入 LED2 主循环。

关键 API：

```c
int ns800_app_main(void);
```

## ADC 后台采样

代码位置：

```text
applications/adc/ns800_adc_background.c
applications/adc/ns800_adc_background.h
```

功能说明：

- 将 PG6-PG14 和 PH13 配置为模拟输入。
- 使用 ADCA SOC0-SOC9 作为 10 路采样通道。
- 使用 EPWM1 SOCA 作为 ADC 硬件触发源。
- SOC9 转换结束后触发 ADCINT1。
- ADCINT1 触发 EDMA，将 RESULT0-RESULT9 搬运到 32 帧环形缓冲区。
- CPU 不参与逐帧触发，只负责读取最新帧。

公开 API：

```c
int ns800_adc_background_start(void);
int ns800_adc_background_stop(void);
const rt_uint16_t *ns800_adc_background_latest(rt_uint32_t *seq);
```

控制层默认使用的 ADC 帧通道语义：

```text
CH0  HVDC 电压
CH1  HVDC 电流
CH2  LVDC 电压
CH3  LVDC 电流
CH4  A 相电压
CH5  A 相电流
CH6  B 相电压
CH7  B 相电流
CH8  C 相电压
CH9  C 相电流
```

典型读取方式：

```c
rt_uint32_t seq;
const rt_uint16_t *frame = ns800_adc_background_latest(&seq);
```

返回的 `frame` 包含 `NS800_ADC_BACKGROUND_CHANNELS` 个采样值。

## PWM 输出

代码位置：

```text
applications/ns800_pwm_app.c
applications/ns800_pwm_app.h
```

功能说明：

- 直接调用 SDK EPWM/GPIO API 配置 EPWM8-EPWM13。
- 开关频率为 30 kHz。
- 时基周期为 `NS800_PWM_APP_TBPRD_30KHZ = 6667`。
- 默认占空比为 50%。
- 每个 EPWM 模块的 A/B 输出作为一对输出配置。

引脚映射：

```text
EPWM8A/8B    GPIO74/75
EPWM9A/9B    GPIO76/77
EPWM10A/10B  GPIO78/79
EPWM11A/11B  GPIO80/81
EPWM12A/12B  GPIO82/83
EPWM13A/13B  GPIO85/86
```

三电平 NPC 桥臂映射：

```text
A 相：EPWM8A/8B 为 upper，EPWM9A/9B 为 lower
B 相：EPWM10A/10B 为 upper，EPWM11A/11B 为 lower
C 相：EPWM12A/12B 为 upper，EPWM13A/13B 为 lower
```

公开 API：

```c
int ns800_pwm_app_start(void);
int ns800_pwm_app_stop(void);
int ns800_pwm_app_set_duty(rt_uint32_t epwm_id,
                           rt_uint32_t channel,
                           rt_uint32_t duty_permille);
rt_uint32_t ns800_pwm_app_get_duty(rt_uint32_t epwm_id,
                                   rt_uint32_t channel);
void ns800_pwm_app_write_svm(const svm_dual_out_t *duty);
```

手动设置占空比例子：

```c
ns800_pwm_app_set_duty(8, NS800_PWM_APP_CHANNEL_A, 500);
ns800_pwm_app_set_duty(8, NS800_PWM_APP_CHANNEL_B, 500);
```

SVM 输出写入路径：

```c
svm_dual_out_t duty;
ns800_pwm_app_write_svm(&duty);
```

`ns800_pwm_app_write_svm()` 会把 `upper.ta/tb/tc` 和 `lower.ta/tb/tc`
映射到上面的 ABC 三相桥臂分组。

## 功率控制中断

代码位置：

```text
applications/ns800_power_control.c
applications/ns800_power_control.h
```

功能说明：

- 使用 EPWM2 作为独立控制定时器。
- 默认控制频率为 10 kHz。
- EPWM2 中断中读取最新 ADC 帧，执行 SVM 控制步进，然后更新
  EPWM8-EPWM13 的 compare 值。
- 记录 ISR 计数、ADC miss 计数、fault flags 和最近一次控制输出。

公开 API：

```c
int ns800_power_control_start(void);
int ns800_power_control_stop(void);
const ns800_power_control_status_t *ns800_power_control_get_status(void);
```

状态读取示例：

```c
const ns800_power_control_status_t *status;
status = ns800_power_control_get_status();
```

## 功率分配与 SVM 算法

代码位置：

```text
applications/pd_svm/
```

主要文件：

```text
svm_types.h        算法共享类型定义。
svm_config.h       默认控制参数和常量。
svm_pi.c/.h        PI 控制器。
svm_transform.c/.h Clarke/Park 变换和角度生成辅助函数。
svm_power_alloc.c/.h
                   双端口功率分配和 SVM 分配逻辑。
svm_control.c/.h   完整电压/电流控制步进。
svm_port.c/.h      ADC/PWM 应用层代码与 SVM 算法之间的硬件适配层。
```

重要类型：

```c
svm_dual_out_t
svm_control_config_t
svm_control_state_t
svm_control_output_t
```

主要算法入口：

```c
void svm_control_default_config(svm_control_config_t *cfg);
void svm_control_init(svm_control_state_t *state,
                      const svm_control_config_t *cfg);
svm_status_t svm_control_step(svm_control_state_t *state,
                              const svm_control_config_t *cfg,
                              const svm_abc_f32_t *ac_voltage_abc,
                              const svm_abc_f32_t *ac_current_abc,
                              float dc_upper_voltage,
                              float dc_lower_voltage,
                              bool reset_pi,
                              svm_control_output_t *out);
svm_status_t svm_control_isr_step(svm_control_state_t *state,
                                  const svm_control_config_t *cfg,
                                  svm_control_output_t *out);
```

默认控制常量位于 `svm_config.h`，包括：

```text
SVM_CONTROL_PERIOD_S = 1.0e-4
SVM_DEFAULT_FREQ_HZ  = 50 Hz
SVM_DEFAULT_UD_REF   = 110
SVM_DEFAULT_UQ_REF   = 0
```

当前 ADC 标定仍是 `svm_port.c` 中的原始值直通映射。模拟前端比例确认后，
应在该文件中更新硬件增益和偏置。

## 按键

代码位置：

```text
applications/button/
```

功能说明：

- 使用已移植的 MultiButton 核心代码 `multi_button.c/.h`。
- 创建永久存在的 `buttons` 线程。
- 扫描周期为 5 ms。
- GPIO12-GPIO18 配置为上拉输入。
- 按键有效电平为低电平。

全局参数：

```c
extern volatile float ns800_param_xi;
extern volatile rt_int32_t ns800_param_speed;
extern volatile rt_int32_t ns800_param_force_ma;
```

默认值：

```text
xi    = 0.5
speed = 2000
F     = 500 mA
```

按键分配：

```text
GPIO12  xi++
GPIO13  xi--
GPIO14  speed++
GPIO15  speed--
GPIO16  F++
GPIO17  F--
GPIO18  reset xi/speed/F
```

公开 API：

```c
int ns800_button_app_start(void);
void ns800_button_app_reset_params(void);
```

这些变量目前由 OLED 显示线程读取和显示，尚未接入 SVM 业务控制逻辑。

## OLED 与 I2C1

代码位置：

```text
applications/oled/
```

I2C1 应用层：

```text
ns800_i2c1_app.c/.h
```

功能说明：

- GPIO91 配置为 `I2C1_SDA`。
- GPIO92 配置为 `I2C1_SCL`。
- 使用 SDK polling I2C 传输。
- 默认 I2C 速率为 400 kHz。

公开 API：

```c
int ns800_i2c1_app_start(void);
int ns800_i2c1_app_write(rt_uint8_t addr7,
                         const rt_uint8_t *data,
                         rt_uint32_t len);
```

SH1106 驱动层：

```text
ns800_sh1106_oled.c/.h
```

功能说明：

- 驱动 128x64 SH1106 OLED，I2C 地址为 `0x3C`。
- 命令控制字为 `0x00`，数据控制字为 `0x40`。
- 在 RAM 中维护 framebuffer。
- 按 8 page x 128 column 刷屏。
- 内置最小 6x8 ASCII 绘制路径，用于状态文字显示。

公开 API：

```c
int ns800_sh1106_oled_start(void);
void ns800_sh1106_oled_clear(void);
void ns800_sh1106_oled_flush(void);
void ns800_sh1106_oled_draw_pixel(rt_uint32_t x,
                                  rt_uint32_t y,
                                  rt_bool_t on);
void ns800_sh1106_oled_draw_string(rt_uint32_t x,
                                   rt_uint32_t y,
                                   const char *text);
```

显示应用层：

```text
ns800_display_app.c/.h
```

功能说明：

- 创建 `oled` 线程。
- 每 200 ms 刷新一次。
- 默认显示：
  - `NS800 PD FOC`
  - `xi`
  - `speed`
  - `F(mA)`

公开 API：

```c
int ns800_display_app_start(void);
```

自定义显示示例：

```c
ns800_sh1106_oled_clear();
ns800_sh1106_oled_draw_string(0, 0, "NS800");
ns800_sh1106_oled_flush();
```

## LED

代码位置：

```text
applications/ns800_led_app.c
applications/ns800_led_app.h
```

功能说明：

- LED1 在子线程中运行，每 500 ms 翻转一次。
- LED2 在主循环中运行，每 1000 ms 翻转一次。
- 使用现有 `BOARD_LED1_PIN` 和 `BOARD_LED2_PIN`。

公开 API：

```c
int ns800_led_app_start(void);
void ns800_led_app_loop(void);
```

## 后续硬件调试注意事项

- 在 `applications/pd_svm/svm_port.c` 中更新 ADC 物理量增益和偏置。
- 在 `applications/pd_svm/svm_config.h` 中更新 SVM 控制增益和参考值。
- I2C/OLED 操作必须保持在显示线程中，不要放入 EPWM2 ISR。
- `ns800_pwm_app.c` 中的功率 PWM 桥臂映射注释应与实际逆变器接线保持一致。
- 如果 OLED 在 400 kHz 下不稳定，可将
  `applications/oled/ns800_i2c1_app.c` 中的 `NS800_I2C1_BAUDRATE_HZ`
  降为 `100000UL`。
