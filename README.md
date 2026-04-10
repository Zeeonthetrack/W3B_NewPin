# W3B_NewPin

## 已完成电控中期任务：6/6

## 项目介绍
W3B_NewPin 是华南理工大学歪比巴卜队的一个基于 STM32F103 的嵌入式控制项目，使用 STM32CubeMX 生成的 HAL 框架与 CMake 构建。主程序通过 UART 接收摇杆数据包，解析后驱动左右两路电机 PWM，并带有通信超时的安全停机逻辑。工程同时包含可选的 OLED 驱动库（当前在主程序中注释未启用）。最终成果以小车呈现。

【仅为中期验收成果，最终版本可能修改引脚定义】

## 技术栈
- C11
- STM32 HAL (STM32CubeMX 生成)
- CMake + Ninja
- GNU Arm Embedded Toolchain (arm-none-eabi)

## 环境要求
- CMake 3.22 或更高版本
- Ninja
- GNU Arm Embedded Toolchain (arm-none-eabi-gcc 等工具在 PATH 中)
- 目标硬件：STM32F103（Cortex-M3）

## 安装步骤
1. 安装 GNU Arm Embedded Toolchain，并确保 `arm-none-eabi-gcc` 在 PATH 中。
2. 安装 CMake 与 Ninja。
3. 进入项目根目录并配置构建预设：
   ```bash
   cmake --preset Debug
   ```
4. 构建项目：
   ```bash
   cmake --build --preset Debug
   ```

## 运行方式
1. 通过构建命令生成可执行文件与产物：
   - `build/Debug/W3B_NewPin.elf`
   - `build/Debug/W3B_NewPin.hex`
   - `build/Debug/W3B_NewPin.bin`
2. 使用你常用的 STM32 下载工具将 `hex` 或 `bin` 文件烧录到目标板。
3. 上电后，固件会初始化 GPIO、USART2、TIM1/TIM2/TIM3/TIM4、I2C2。
4. 初始化完成后会启动电机相关 PWM 与编码器，并初始化 PCA9685（地址 `0x40`，`50Hz`）。

## 中期验收视频
项目根目录包含 `中期电控.zip`，内含 3 个演示视频：
- `舵机.mp4`：舵机控制演示。
- `激光.mp4`：激光输出相关演示。
- `蓝牙小车.mp4`：蓝牙小车控制演示。

## 目录结构说明
- Core/Inc: 头文件（如 `main.h`, `gpio.h`, `tim.h`, `usart.h`）
- Core/Src: 主程序与外设初始化（如 `main.c`, `gpio.c`, `tim.c`, `usart.c`）
- Drivers: STM32 HAL 与 CMSIS 驱动库
- Lib/oled: OLED 显示库（已加入工程，但主程序内默认未启用）
- cmake: 工具链与 CubeMX CMake 生成文件
- build: 构建输出目录

## 使用示例
### UART 控制数据包格式（代码实装）
主程序当前实现了以下 4 类控制帧：

1. 摇杆控制帧（文本）
```
[j,Lx,Ly,Rx,Ry]
```
- `Lx/Ly/Rx/Ry` 取值范围为 `-100` 到 `100`
- 其中 `Ly/Ry` 控制左右轮目标速度，`Lx` 控制原地旋转，`Rx` 控制麦克纳姆横移

2. 舵机/速度控制帧（文本）
```
[s,id,x]
```
- `id=1/3/4/5`：控制对应舵机角度
- `id=2`：设置键控前进/后退速度档位（百分比）

3. 键控动作帧（文本）
```
[k,x,y]
```
- `x`：`w/s/j/l/q/a`
- `y`：`d`(按下/开始) 或 `u`(松开/停止)

4. 云台控制帧（二进制）
```
[0xAA, panL, panH, tiltL, tiltH, checksum, 0x55]
```
- `checksum` 为 `panL ^ panH ^ tiltL ^ tiltH`

> 若超时 `500ms` 未接收到有效遥控输入，轮速目标自动回零（安全停机）。

### 关键引脚与外设
- UART: USART2，PA2(TX) / PA3(RX)，波特率 115200
- 电机 PWM: TIM3_CH1(PA6)、TIM3_CH2(PA7)、TIM1_CH1N(PB13)、TIM1_CH2N(PB14)
- 编码器: TIM2_CH1/CH2(PA0/PA1)，TIM4_CH1/CH2(PB6/PB7)
- 舵机总线: I2C2，PB10(SCL) / PB11(SDA)，PCA9685 地址 `0x40`
- 电机方向控制 GPIO: PB15、PA8、PB5、PB4、PB3

