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

## 5. 找到帧以后怎样使用（MC02-0123）

[vision_protocol.h](../../modules/master_machine/vision_protocol.h) 区分“计算 CRC”与“检查帧”：
计算入口的长度不含待追加的 CRC，检查入口的长度包含末尾两字节。
找帧入口逐字节前移，头或 CRC 不对就继续，找到第一份完整正确帧立即返回。
失败时命令输出不改写，错误计数则先清零；调用方不能把上一份输出当成本次找到的帧。

[master_process.h](../../modules/master_machine/master_process.h) 在采用数据前按真正接收时刻检查 50 ms 时限，
读取和发送时也会检查旧命令是否到时。角度使用 rad，角速度使用 rad/s，进入电机控制前另做换算。
CRC 正确只说明字节检查通过，模式、数字、机械角度和当前是否允许控制仍由后续代码检查。

同目录的 [seasky_protocol.h](../../modules/master_machine/seasky_protocol.h) 保留另一种 A5 帧头格式，
不能和 SP 帧混用。它没有接收长度或输出容量参数，调用方须先保证整帧及空间足够，
帧头长度至少包含两字节标志；校验失败返回 0，标志和数据输出保持原值。

## 6. 裁判系统 CRC 和图形发送（MC02-0123）

[crc_ref.h](../../modules/referee/crc_ref.h) 的 CRC-8 和 CRC-16 接口也分成“计算后写入”和“只检查”两类。追加 CRC 时，传入长度包含预留的校验字节；CRC-8 写末尾1字节，CRC-16 按低字节、高字节写末尾2字节。检查接口不会改缓冲区，计算接口只返回数值，也不会帮调用方追加字节。它们属于裁判帧接口，不能因名称相似就和其他模块的 CRC 函数互换。

[referee_UI.h](../../modules/referee/referee_UI.h) 的 `UIGraphRefresh()` 只支持1、2、5或7个图形，图形通过可变参数按值传入；`cnt` 与后续参数数量必须对应。函数根据数量选择协议命令，再把每个图形放入帧中并追加 CRC-16。[RefereeSend()](../../modules/referee/rm_referee.h) 用 DMA 发送后还会等待115 ms；它没有返回发送状态，调用方不能把函数返回当作发送成功的证明。
