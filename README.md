# W3B_NewPin

## 项目介绍
【仅为中期验收成果，最终版本可能修改引脚定义】
W3B_NewPin 是华南理工大学歪比巴卜队的一个基于 STM32F103 的嵌入式控制项目，使用 STM32CubeMX 生成的 HAL 框架与 CMake 构建。主程序通过 UART 接收摇杆数据包，解析后驱动左右两路电机 PWM，并带有通信超时的安全停机逻辑。工程同时包含可选的 OLED 驱动库（当前在主程序中注释未启用）。最终成果以小车呈现。

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
3. 上电后，固件会初始化 GPIO、USART2、TIM3、TIM4，并启动 PWM 输出。

## 中期验收视频
压缩包内包含 3 个演示视频，用于展示中期电控功能：
- 舵机.mp4：展示舵机的控制与动作效果。
- 激光.mp4：展示激光输出的开关与响应。
- 蓝牙小车.mp4：展示通过蓝牙控制小车的运动效果。

## 目录结构说明
- Core/Inc: 头文件（如 `main.h`, `gpio.h`, `tim.h`, `usart.h`）
- Core/Src: 主程序与外设初始化（如 `main.c`, `gpio.c`, `tim.c`, `usart.c`）
- Drivers: STM32 HAL 与 CMSIS 驱动库
- Lib/oled: OLED 显示库（已加入工程，但主程序内默认未启用）
- cmake: 工具链与 CubeMX CMake 生成文件
- build: 构建输出目录

## 使用示例
### UART 摇杆数据包格式
主程序解析形如 `[j,Lx,Ly,Rx,Ry]` 的数据包，范围为 -100 到 100：
```
[j,0,50,0,-50]
```
- `Ly` 映射到左电机速度（TIM3_CH1）
- `Ry` 映射到右电机速度（TIM4_CH3）
- 若 500 ms 内未接收到有效数据包，将自动将两路电机速度置零

### 关键引脚与外设
- UART: USART2，PA2(TX) / PA3(RX)，波特率 9600
- PWM: TIM3_CH1(PA6) 与 TIM4_CH3(PB8)
- 电机控制 GPIO: PB13/14/15、PA8、PB3/4/5
- 指示/激光输出: PA0
