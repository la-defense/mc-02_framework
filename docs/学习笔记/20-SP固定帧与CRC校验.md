# SP 固定帧与 CRC 校验

本文记录 MC-02 与 `sp_vision_25` 的固定帧约定，以及如何用纯主机测试守住两端的协议边界。

## 1. 两个方向，两种长度

| 方向 | 结构 | 长度 | 典型内容 |
|---|---|---:|---|
| MC-02 → Vision | `Vision_Send_s` / `GimbalToVision` | 43 字节 | 模式、姿态四元数、云台角度与弹速/计数 |
| Vision → MC-02 | `Vision_Recv_s` / `VisionToGimbal` | 29 字节 | 控制模式与 yaw/pitch 位置、速度、加速度 |

帧头为两个字节 `SP`。双方结构加上 packed 属性和 `static_assert` 能检查大小，但字段顺序、浮点表示、大小端和 CRC 覆盖范围仍必须由双方共同遵守。

## 2. 从字节流里找帧

UART/USB 回调拿到的是字节区间，不保证从帧头开始。MC-02 从缓冲区起点逐字节扫描 `SP`，每找到一个候选位置，就确认剩余长度足够，再校验 CRC。完整且校验通过后才复制到接收状态并更新时间；坏 CRC 会增加错误计数，但不能刷新链路在线状态。短于一帧时直接拒绝。

当前解析函数 `VisionProtocolFindCommandFrame()` 把这个动作抽成不依赖 HAL 的纯 C 函数，因此可以由主机 GCC 直接测试：

```text
长度不足 → 拒绝
完整但没有 SP → 继续扫描
SP + CRC 不匹配 → 计错误，继续扫描
找到有效帧 → 返回帧；只由调用方发布并刷新在线状态
```

## 3. CRC 不是认证

CRC 可以发现覆盖范围内常见的意外传输损坏，但不能证明发送者是谁，也不防止恶意构造。这个工程的线格式要求完整保留当前 CRC 算法、初值、反射方向、覆盖长度和两个 CRC 字节的顺序；“都叫 CRC16”不代表结果相同。

MC-02 和视觉代码的当前实现使用同一位运算表述，主机测试将 `123456789` 的结果固定为 `0x6f91`。这是仓库实际协议实现的回归向量；更换 CRC 库前先核对 MCU 端算法和已抓取的真实帧。

CRC16 数值在协议帧尾按低字节、再高字节排列。读回时也按相反步骤组合为 16 位数值。测试应同时验证已知向量、完整合法帧、帧头错误、载荷翻转、短帧，以及坏帧后紧跟合法帧的重同步行为。

## 4. 代码与验证入口

- 固件结构、CRC 和扫描器：`modules/master_machine/vision_protocol.h/.c`
- 固件状态、计数与原子快照：`modules/master_machine/master_process.c`
- 主机用例：`tests/host/vision_protocol_test.c`
- 主机命令：`cmake -S tests/host -B build/host-tests -G Ninja`，之后构建并运行 `ctest --test-dir build/host-tests --output-on-failure`
- 对应离线课程：[SP 帧与 CRC](../课程/自瞄链路/lessons/0001-sp-frame-crc.html)、[SP 练习](../课程/自瞄链路/exercises/01-sp-frame-crc/problem/readme.md)

主机测试证明 parser 对这些输入按约定工作；它不代替 USB 传输实测、固件烧录或对真实硬件抓帧。
