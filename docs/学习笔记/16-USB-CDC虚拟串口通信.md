# 16 USB CDC 虚拟串口通信（写给零基础）

> 本篇回答的问题：
> 1. USB 和"物理串口"到底差在哪？为什么插上板子就能多出一个 COM 口？
> 2. CDC 是什么？"虚拟串口"是怎么装出来的？
> 3. `USBD_CDC_ReceivePacket()` 为什么要"重新武装"？忘了会怎样？
> 4. 为什么 USB 接收回调里不能久留？
> 5. 发送时报 `USBD_BUSY` 是什么情况？怎么处理？
> 6. 本工程的视觉链路为什么选 VCP 而不是物理串口？
>
> 对应代码：`bsp/usb/bsp_usb.c`、`USB_DEVICE/App/usbd_cdc_if.c`、
> `modules/master_machine/master_process.c`。

---

## 1. USB 和物理串口的本质区别

虽然用起来都是"打开一个 COM 口收发数据"，底层完全是两回事：

| | 物理串口（UART） | USB |
|---|---|---|
| 通信模型 | 两个设备平等，各自收发 | **主机（Host）/ 设备（Device）**，电脑是主机，板子是从机 |
| 谁发起通信 | 双方随时都能发 | **永远由主机发起**，设备只能"准备好等着"或"响应" |
| 数据单位 | 一个字节一个字节的连续流 | **包（Packet）**，一次最多 64 字节（全速模式） |
| 时钟 | 双方各自按约定波特率 | 主机通过差分线上的信号编码提供时钟 |
| 端点 | 不存在 | 每个方向有若干**端点（Endpoint）**，各自独立编号 |
| 能不能"随便插拔" | 要重新初始化 | 协议本身支持枚举/热插拔 |

**"设备不能主动发起"这一条最关键**：它决定了 USB 接收必须**提前把缓冲区挂上去**，
等主机来填。这就是下面要讲的"重新武装"。

---

## 2. CDC 是怎么把 USB 变成串口的

USB 协议里有一类叫 **CDC（Communication Device Class）**，专门用来描述"通信类设备"。
配合一个叫 **ACM（Abstract Control Model）** 的子类，就能让 USB 设备表现得像一台调制解调器/串口。

板子这侧要做的就是：

1. 描述自己是 CDC 设备（接口描述符）；
2. 提供一对批量端点：一个 IN（设备→主机）、一个 OUT（主机→设备）；
3. 实现 CDC 规定的几个控制请求（设置波特率、线路状态等）——**注意这些参数是"意思一下"**，USB 实际速度跟"波特率"无关，上位机随便设都能通。

电脑那侧，操作系统看到 CDC 设备就加载 `usbser`（Windows）/`cdc_acm`（Linux）驱动，
于是"多出来一个 COM 口"。**这就是虚拟串口 VCP（Virtual COM Port）**。

> 所以 VCP 的"波特率"只是个摆设，实际速度取决于 USB：全速（FS）模式下理论 12Mbps，
> 实际批量传输能跑到几百 KB/s ~ 1MB/s，**比 921600 的物理串口还快一个数量级**。

---

## 3. 接收：为什么要"重新武装"

设备侧接收数据的过程是这样的：

```
① 设备把一块缓冲区交给端点: USBD_CDC_SetRxBuffer(&hUsb, buf);
② 告诉 USB 外设"我准备好了, 主机可以往里发了": USBD_CDC_ReceivePacket(&hUsb);
③ 主机发来一包数据 → 填进 buf → 触发 USB 中断
④ 在回调 CDC_Receive_HS(Buf, Len) 里处理数据
⑤ 处理完必须再做一次 ①② —— 这叫"重新武装"
```

**第 ⑤ 步忘了会怎样**：端点没有挂缓冲区，主机再发数据时设备直接不响应（NAK），
数据就一直重试或丢弃 —— 表现为"**只能收到第一包**"。

这也是为什么本工程原来的代码长这样：

```c
static int8_t CDC_Receive_HS(uint8_t *Buf, uint32_t *Len)
{
    usb_rx_callback((uint16_t)(*Len));      /* 处理这一包 */
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);   /* 重新挂缓冲 */
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);          /* 重新武装 */
    return USBD_OK;
}
```

---

## 4. 为什么回调里不能久留（本次改造的动机）

`CDC_Receive_HS()` 是在 **USB 中断上下文**里被调用的。看上面那段代码的问题：
`usb_rx_callback` 指向的是视觉的 `DecodeVision()`，而它要做：

1. 在收到的字节里**滑动**，找 `'S' 'P'` 帧头；
2. 每找到一个候选就**算一遍 CRC16** 校验；
3. 校验通过才整包解析、更新时间戳、喂守护进程。

在 200Hz 的视觉数据流下，这就是**每秒 200 次、每次几十微秒**的中断开销；
如果上位机发得快或者数据里有噪声（一直在滑动找帧头、一直算 CRC），耗时还会更长。
中断里待得越久，其它中断（CAN、串口、定时器）被推迟得越厉害。

**改造后的分工**（和《12》《15》完全一致）：

```c
/* 中断: 只把这一包拷进队列 */
static int8_t CDC_Receive_HS(uint8_t *Buf, uint32_t *Len)
{
    VCPQueuePush(Buf, (uint16_t)(*Len));            /* 拷贝入队, 几微秒 */
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);          /* 立刻重新武装 */
    return USBD_OK;
}

/* 任务(1kHz): 逐包取出来再解析 */
void VCPProcessRx(void)
{
    while (队列非空)
    {
        memcpy(端点缓冲, 队列项.data, len);   /* 拷回原缓冲, 模块回调零改动 */
        s_rx_cbk(len);                        /* = DecodeVision(len) */
        推进 tail;
    }
}
```

**为什么也要拷贝**：USB 端点缓冲（本工程是 `UserRxBufferHS`）是**复用的**——
`CDC_Receive_HS` 返回前就把同一个 buffer 重新挂上去了，
下一包数据随时可能盖进来。只存指针给任务，任务读到的就是半新半旧的数据。

> 这和《15》里 DMA 串口的情况**一模一样**：
> 只要生产者是硬件、消费者是任务，中间就必须有一份拷贝。

---

## 5. 发送：`USBD_BUSY` 是怎么回事

发送侧是同步的，代码长这样：

```c
uint8_t CDC_Transmit_HS(uint8_t *Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (hcdc->TxState != 0)
        return USBD_BUSY;              /* 上一包还没发完! */
    USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
    USBD_CDC_TransmitPacket(&hUsbDeviceHS);
    return USBD_OK;
}
```

**它是异步的**：`TransmitPacket` 只是把数据交给端点，真正的传输在后台由 USB 外设完成，
完成前 `TxState != 0`。这期间再调一次发送就会拿到 `USBD_BUSY`。

**所以连续发送要注意**：

| 做法 | 适合场景 |
|---|---|
| 发送前判断返回值，忙就跳过这一帧（丢帧） | 状态类数据（视觉上行就是这种，丢一帧无所谓，下一帧 5ms 后就来） |
| 用一个发送队列，等 `TxCpltCallback` 再发下一帧 | 不能丢的数据（目前工程里不需要） |
| **传的 buffer 要用静态/全局变量** | 必须：`TransmitPacket` 是异步的，用栈上临时变量会在发完前失效 |

本工程的视觉上行 `VisionSend()` 用的是第一种：200Hz 够用，忙就丢。

---

## 6. VCP 与物理串口的取舍（为什么视觉选 VCP）

| 维度 | 物理串口 | VCP |
|---|---|---|
| 带宽 | 115200~921600（约 11~92 KB/s） | 几百 KB/s ~ 1MB/s |
| 延迟 | 稳定、可预测 | 稍高但抖动小（主机轮询周期决定） |
| 占用引脚 | 2 根（TX/RX）+ 可能的电平转换 | 只占 USB 那两根 D+/D- |
| 上位机适配 | 需要串口线/模块 | **一根 USB 线就够了** |
| 抗干扰 | 长线容易受干扰 | USB 差分、协议自带重传 |

视觉数据传输量大（200Hz × 每帧几十字节，还要接收上位机的瞄准指令），
而且机器人上本来就要接一根 USB 线做调试 —— **用 VCP 等于"调试口兼数据口"**，
省一路串口和一根线。这就是本工程视觉链路选 VCP 的原因。

裁判系统则相反：它的硬件接口**只提供串口**（RS232/TTL），所以必须走 UART。

---

## 7. 速查表

| 问题 | 答案 |
|---|---|
| 为什么插上板子就有 COM 口 | 板子把自己描述成 CDC 设备，系统加载 usbser/cdc_acm 驱动 |
| VCP 的"波特率"有意义吗 | 没有，只是让上位机开心的参数，实际速度看 USB |
| 为什么只能收到第一包 | 忘了在回调里 `SetRxBuffer` + `ReceivePacket` 重新武装 |
| 为什么接收回调里不能久留 | 那是 USB 中断上下文，久留会推迟其它中断 |
| 把数据指针丢给任务行不行 | **不行**，端点缓冲会被下一包覆盖，必须拷贝 |
| 发送返回 `USBD_BUSY` 怎么办 | 说明上一包没发完；状态类数据直接跳过，要紧的数据用发送队列 |
| 发送用的 buffer 能是局部变量吗 | **不能**，发送是异步的，局部变量会先失效 |
| 视觉为什么用 VCP 不用串口 | 带宽高一个数量级，而且省一路串口/一根线 |
