# 基于 STM32F103 的 TIM6 获取 1527 编码遥控器的键值

MCU 型号是 STM32F103RET6，采用定时器 TIM6 解码 1527 的例程。

### 硬件

![屏幕截图 2023-04-26 141055](https://user-images.githubusercontent.com/117444566/234498625-8193c0b8-e8a5-4e34-a780-456585271f8c.png)

### 注意

- 如果出现中文乱码，请修改文件编码为 GB18030
- 工程使用 IDE 为 ARM Keil 5.14
- 只适用于 1257 编码的 433 MHz 遥控器
- 获取键值成功后，保存在 rf_data[2] 低 4 位
