# 03 非阻塞通信与 CAN 收发技术细节（写给零基础）

## 0. 一句话总结

**非阻塞 = "能发就发，发不了就记一笔、下个周期再来"，绝不在原地干等。**
CAN 外设内部有一块自己的 RAM 和硬件发送队列，配合"发送完成中断"，就能做到几乎不占 CPU 的发送；而"原地等"会把控制任务的时间预算吃光——我们在 MC-02 上实测到 motor 任务 98.9% 的时间都耗在等 CAN 上。

---

## 1. 先建立直觉：三种 I/O 模型

任何"和外部设备打交道"的操作，CPU 只有三种参与方式：

| 模型 | 做法 | 生活类比 | CPU 占用 | 实时性 |
|---|---|---|---|---|
| **阻塞式 / 轮询（blocking / polling）** | CPU 反复检查"好了没"，好了才继续 | 站在银行柜台前一直等到办完 | 100% 被占住 | 简单但最差 |
| **中断式（interrupt, IT）** | 交出控制权，硬件完成后打断 CPU 去执行一小段处理函数（ISR） | 拿号去旁边坐着，叫号再过去 | 只在事件发生时占用 | 好 |
| **DMA** | 交给专门的搬运硬件，CPU 完全不参与数据搬移 | 找代办，你连椅子都不用坐 | 几乎为 0 | 最好（但要硬件支持） |

关键概念：

- **ISR（中断服务函数）里不能做耗时的活**：不能 `printf`、不能 `malloc`、不能长时间循环。ISR 只适合"搬运 + 置标志"，复杂计算留给任务。
- **"等"也是 CPU 开销**：`while(...)` 空转和真正算东西一样吃电、一样占时间片，只是没产出。
- **非阻塞的本质**：把"等"从代码里删掉，改成"试试看 + 记状态 + 下次再来"。

---

## 2. STM32 的 FDCAN 外设到底长什么样

新手最容易卡在"我不知道硬件帮我做了什么"。把 FDCAN 想成**一个独立的小电脑**：

- 它有**自己的寄存器**（配置、状态、中断）；
- 它有**自己的内存**，叫**消息 RAM（Message RAM）**，是 MCU 内部专门划给 CAN 的一块 RAM；
- 它**自己会按 CAN 的位时序把帧一位一位发到线上**，不需要 CPU 逐位参与。

消息 RAM 被分成若干区（H7 的 FDCAN 是这样）：

```
标准滤镜(Filter) | 扩展滤镜 | RxFIFO0 | RxFIFO1 | RxBuffer | TxBuffer | TxFIFO/Queue | TxEventFIFO
```

### 2.1 发送一帧，硬件做了哪些事

1. CPU 把**帧头 + 数据**写进消息 RAM 里的某个 TX 槽位（HAL 里叫 `AddMessageToTxFifoQ`）；
2. CPU 置位该槽的**发送请求位**（TXBAR）；
3. 外设按位时序把帧送上总线，然后在帧尾等接收方的 **ACK 位**；
4. 收到 ACK → 该槽标记完成 → 触发**发送完成中断**（TCE）；如果整个 FIFO 空了 → 触发 **Tx FIFO Empty**（TFE）；
5. 如果没人 ACK（总线上没有其他节点），这次发送**失败**，错误计数增加（见第 7 节）。

**重要**：第 1 步只是"把信投进信箱"，**不等待发送成功**。真正的等待发生在硬件内部。所以 CPU 完全可以投完就走人——这才是"非阻塞发送"能成立的根本原因。

### 2.2 三种发送缓冲组织方式

| 方式 | 行为 | 我们用哪个 |
|---|---|---|
| **Dedicated Tx Buffer** | 每个 buffer 固定绑一个消息 ID，优先级最高 | 不用 |
| **Tx FIFO** | 先进先出，按写入顺序发送 | ✅ 我们用这个 |
| **Tx Queue** | 按**消息 ID 优先级**发送，ID 越小越先发 | 不用 |

可以配置缓冲深度（我们原来是 4 个槽位，本次改成 16 个）。深度越大，突发情况下越不容易"信箱满"。

### 2.3 接收一帧，硬件做了哪些事

1. 总线上的帧先过**滤镜（Filter）**：滤波器决定"这帧我要不要"；
2. 要 → 写入 **RxFIFO0** 或 **RxFIFO1**（硬件队列），然后触发 `RX_FIFO0_NEW_MESSAGE` 中断；
3. CPU 在 ISR 里用 `HAL_FDCAN_GetRxMessage()` 把帧**取走**（这一步千万不能忘：不取走 FIFO 会满，之后新帧全部丢失）；
4. 取走之后，CPU 根据 ID 找到"这帧属于哪个设备"，调用对应的解析函数。

### 2.4 中断源为什么要"能不开就不开"

FDCAN 的接收侧可以开这些中断：`NEW_MESSAGE`（来了新帧）、`WATERMARK`（FIFO 填充达到阈值）、`FULL`（FIFO 满）、`MESSAGE_LOST`（因为 FIFO 满丢过帧）。

我们原来的代码**四个全开**，而中断处理函数把 `NEW_MESSAGE / WATERMARK / FULL` 当成同一件事处理 → **同一个事件会进好几次中断**，白白多花 CPU。这就是"中断源精简"的动机：只留

- `NEW_MESSAGE`（真正需要的事件）
- `MESSAGE_LOST`（丢帧诊断）

---

## 3. 为什么经典 CAN 的发送"用不上 DMA"（很常见的误解）

DMA 的工作模型是：**内存 ↔ 外设数据寄存器（DR）**逐字节搬运。UART、SPI 都是这种模型（数据必须一个个写进 DR），所以它们用 DMA 非常划算。

**FDCAN 不是 DR 模型**：数据放在消息 RAM 里，外设自己去读。CPU 只需要"写消息 RAM + 置请求位"，然后就没事了。

所以当有人说"把 CAN 发送改成 IT/DMA"时，落到 STM32 上真正的意思是：

> **用硬件 Tx FIFO/Queue 当缓冲区 + 发送完成中断做统计 + 调用处绝不阻塞。**

这三件事合起来，效果就等价于"发送不占 CPU"。

---

## 4. 我们原来的写法为什么把系统拖垮（真实数据）

改造前的代码（简化）：

```c
while (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0) {      /* 信箱满就一直等 */
    if (DWT_GetTimeline_ms() - start > timeout) {        /* 1ms 超时 */
        LOGWARNING("CAN MAILbox full! ...");
        return 0;
    }
}
HAL_FDCAN_AddMessageToTxFifoQ(handle, &conf, buf);
```

### 4.1 时间账

- CAN 1Mbps，一帧（8 字节数据 + 位填充）约 **130µs** 上线；
- 总线上**没有其他节点 → 没有 ACK → 这一帧失败**；只有失败之后，那个槽位才会释放（约 150µs）；
- motor 任务每 1ms 要发 **2~3 帧**（底盘一组、云台一组、摩擦轮/拨弹一组）；
- 一旦信箱被塞满，之后每次调用都要**等满 1ms 超时**才返回 → 一个 tick 白等 **2~3ms**（而 tick 只有 1ms）。

### 4.2 实测（30 秒窗口）

| 指标 | 数值 |
|---|---|
| CAN 发送段耗时 | **3068µs / 次**（占 DJIMotorControl 的 98.9%） |
| 真正的 PID 计算 | 34.2µs / 次 |
| 30 秒内发送次数 | 22698 次，**全部超时失败** |
| 自旋轮数 | 462 万轮 |
| 后果 | motor 任务只能跑到 250Hz（期望 1kHz），整机 58% CPU 被"等"吃掉 |

### 4.3 更糟的一层：BusOff 死循环

失败会让错误计数疯狂增长，很快把节点打进 **BusOff**（见第 7 节）。BusOff 之后外设**彻底停止发送** → 信箱永远不空 → 每次调用必然 1ms 超时 → 死循环自我放大。
同时每次超时都打一条 `LOGWARNING`，把只有 1KB 的 RTT 日志缓冲刷爆（日志格式化 + 写缓冲本身也吃 CPU）。

---

## 5. 改成非阻塞：具体写了什么

### 5.1 发送：满了就丢，绝不等待

```c
uint8_t CANTransmit(CANInstance *ins, float timeout /* 保留但不再使用 */)
{
    probe_start = DWT_ProbeStart();

    if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0u) {   /* 信箱满 */
        stats->tx_drop++;                                /* 记一笔 */
        LogRateLimited(...);                             /* 限速打印, 不刷屏 */
        DWT_ProbeDone(&can_prof_tx, probe_start);
        return 0;                                        /* 立刻返回, 不等待 */
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &conf, buf) != HAL_OK) {
        stats->tx_error++;
        return 0;
    }
    stats->tx_ok++;
    return 1;
}
```

**为什么"丢一帧"比"等 1ms"好？**

- 丢一帧的后果：这个电机在接下来的 1ms 里继续执行上一帧的指令（电流/速度给定是连续的），几乎没有影响；下一周期会自动重发；
- 等 1ms 的后果：整个 motor 任务这一轮延迟 1~3ms，**所有**电机一起被拖慢，还能把更高层的控制节奏打乱；
- 一句话：**丢一帧是局部小损失，等待是全系统大损失。**

### 5.2 缓冲深度与发送完成统计

- TX FIFO 深度 4 → 16：突发时多给硬件一点缓冲；
- 打开 `FDCAN_IT_TX_FIFO_EMPTY`（队列清空）与 `FDCAN_IT_TX_COMPLETE`（单帧完成）中断，在回调里只做计数：

```c
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) { bus->tx_complete++; }
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t indexes)
{ bus->tx_done += __builtin_popcount(indexes); }
```

**注意**：这两个回调在**中断上下文**执行——只允许加法、赋值这类操作。不能打印、不能调用可能阻塞的东西。

### 5.3 接收：中断源精简 + O(1) 查表派发

**问题一**：原来四个接收中断源全开，同一个事件进多次中断 → 只留 `NEW_MESSAGE | MESSAGE_LOST`。

**问题二**：原来每收到一帧，都要遍历所有已注册 CAN 实例，比对"句柄 + rx_id"才找到归属：

```c
for (i = 0; i < idx; ++i)                       /* 9 个实例就是 9 次比较 */
    if (handle == inst[i]->handle && id == inst[i]->rx_id) { ... }
```

每总线 5 个电机、1kHz 反馈 = 每秒 5000 帧 → 每秒 45000 次比较，全在中断里做。

**改法**：注册时建一张查找表，用"ID 的低 9 位"做下标：

```c
static CANInstance *rx_lookup[3][512];          /* [总线][rx_id & 0x1FF] */
...
CANInstance *ins = rx_lookup[bus][id & 0x1FF];
if (ins != NULL && ins->rx_id == id) { 回调(ins); }   /* 命中: 1 次索引 + 1 次校验 */
else { 回退线性查找 }                                  /* 极少数别名冲突才走这里 */
```

- 为什么不做成 2048（11 位完整 ID）的直表：`2048 × 4B × 3 总线 = 24KB`，RAM 太贵；
- 512 张 × 4B × 3 = 6KB，可以接受；低位相同的不同 ID（别名）用"完整 ID 再校验 + 回退线性查找"解决；
- 效果：常见路径从"9 次比较"变成"1 次索引 + 1 次比较"，且**不含循环分支**，更快更稳。

**问题三（顺带修的 bug）**：原来的 ISR 找到匹配实例后**直接 `return`**，FIFO 里剩下的帧这一轮就不处理了 → FIFO 很快满 → 开始丢帧（`MESSAGE_LOST`）。现在处理完一帧**继续排空 FIFO**，直到队列真的空。

---

## 6. 任务与 ISR 的分工原则（本工程约定）

| 工作 | 放在哪 | 理由 |
|---|---|---|
| 把帧从 RxFIFO 取走、拷进实例缓冲 | ISR | 必须尽快腾空 FIFO，否则丢帧 |
| 按 ID 找设备、调用解析回调 | ISR（但必须 O(1)） | 延迟敏感（控制用 1kHz 反馈） |
| 电机 PID、状态机、决策 | 任务 | 需要时间，放 ISR 会拖垮系统 |
| 日志打印、参数保存、BusOff 恢复 | 任务（低频） | 耗时、可能阻塞 |

---

## 7. CAN 的错误状态机：TEC / REC / Error Passive / BusOff

### 7.1 两个计数器

- **TEC**（Transmit Error Counter）：发送错误计数
- **REC**（Receive Error Counter）：接收错误计数

粗略规则（够用版）：

- 发送失败（超时/错误帧）：TEC **+8**
- 发送成功：TEC **-1**
- 接收失败：REC +1；接收成功：REC -1（如果 REC > 127 则 -1 也要满足条件）

### 7.2 三个状态

| 状态 | 条件 | 行为 |
|---|---|---|
| Error Active | TEC、REC ≤ 127 | 正常收发，发现错误时发"主动错误帧" |
| Error Passive | 128 ≤ TEC 或 REC ≤ 255 | 仍能收发，但只能发"被动错误帧"（不打断总线） |
| **BusOff** | TEC > 255 | **停止收发**，从总线上"退出" |

### 7.3 BusOff 之后会发生什么

标准（ISO 11898）规定：节点进入 BusOff 后，要**监听 128 次"11 个连续隐性位"**才允许重新上线（防止一个坏节点一直干扰总线）。
STM32 的实现里，如果 `CCCR.INIT` 被置位（比如协议异常、或软件 `HAL_FDCAN_Stop()`），节点会**停在初始化状态**，必须由软件把它拉回来。

**我们板子的真实情况**：空总线上每帧都失败 → 30 多次失败就 BusOff → 而固件里**根本没有恢复代码** → CAN 永久死掉（LCD 第 4 页那两个 `OFF` 就是这么来的）。

### 7.4 我们的恢复策略

在 100Hz 的 daemon 任务里挂一个 10Hz 的健康检查：

```c
void CANHealthMonitor(void)
{
    for (每个总线) {
        HAL_FDCAN_GetProtocolStatus(h, &ps);
        if (ps.BusOff) {
            bus->busoff++;                        /* 计数 */
            HAL_FDCAN_Stop(h);                    /* 置 CCCR.INIT, 复位错误状态 */
            HAL_FDCAN_Start(h);                   /* 清 INIT, 重新上线 */
            重新使能接收中断;                       /* Stop 之后中断使能可能丢失 */
            bus->busoff_recover++;
            限速打印;
        }
    }
}
```

- 为什么放 10Hz 任务：恢复动作要动好几个寄存器，放 ISR 不合适；10Hz 足够快，也不会自己造成抖动；
- 为什么要计数：BusOff 频繁发生本身就是"总线有问题"的信号（终端电阻、地线、波特率、线缆）；
- 为什么限速打印：BusOff 可能每 100ms 一次，不限速日志会把 CPU 和 RTT 缓冲吃光（我们已经吃过这个亏）。

### 7.5 AutoRetransmission（自动重传）的取舍

| 设置 | 行为 | 适合 |
|---|---|---|
| **开** | 一帧没 ACK 会由硬件一直重发，直到成功或 BusOff | 点对点可靠链路 |
| **关（我们用）** | 失败一次就放弃，下一周期由软件自然重发 | 多节点控制总线：**不会让一个失败帧把 FIFO 占死** |

我们原来的灾难 = **自动重传关闭 + 软件又自己自旋等待超时**：硬件不重传，软件却死等，两头都不对。现在硬件不重传（保持）、软件也不等（改成丢帧计数），才是自洽的组合。

---

## 8. 怎么验证（没有电机也能验证）

| 场景 | 能验证什么 | 方法 |
|---|---|---|
| **空总线（没有其他节点）** | 天然的故障注入：发送失败、BusOff、恢复流程、丢帧计数 | 看探针：`can_prof_tx` 从 3068µs → 几十µs；`tx_drop` 增长；`busoff_recover` 增长但系统不死 |
| **USB-CAN 适配器接上** | 天然的 ACK 源：发送成功路径、TEC 不再累加、抓包看节奏 | 监听能否看到 `0x200/0x1FF/0x2FF` 命令帧；`tx_drop` 不再增长 |
| **适配器模拟电机反馈** | 接收/解析路径、电机在线判定 | 周期发送 `0x201~0x204` 反馈帧 |

复测命令（本工程工具）：

```powershell
# 占用率与分段耗时对比（推荐 30s 以上）
python tools\probe_snapshot.py --seconds 30

# 看 / 抓 RTT 日志（本机 OpenOCD 没编 rtt 命令，工具直读环形缓冲）
python tools\rtt_dump.py --seconds 20
```

### 8.1 本机那台 USB-CAN 适配器的实测结论（2026-09）

- **它不是 slcan/CANable**：对 `\r`、`C\r`、`S8\r`、`O\r` 等 ASCII 命令零响应，而是持续输出**私有二进制协议**；
  帧长 16 字节、同步字 `55 AA`，实测格式（由观测推断）：

  ```
  55 AA | 11 08 | ID_L ID_H | DATA0..7 | xx xx
  ```

- 因此 **python-can 不能直接驱动它**（python-can 支持 pcan/kvaser/slcan/canalystii/… 但都要求厂商规定的协议或 DLL）。
- 但它**能当"只听"的监视器用**：`tools/usbcan_serial_monitor.py` 就是按上面的格式解帧的，
  实测能看到 MCU 发出的 `0x200` 帧（约 1.2~2.2k 帧/s）；**把 MCU 暂停后数据流几乎归零**，
  这条实验反过来证明了"它报的就是总线上的真实帧、且波特率/接线都是对的"。
- 它**不回 ACK**：MCU 侧 TEC 一直涨到 200+（接近 BusOff），说明它要么处于"通道未打开/静默"状态，
  要么就是纯监听设计。要让它参与收发（ACK、发帧），需要厂商上位机/协议文档里的"打开通道"命令。
  这也说明：**用一台不回 ACK 的适配器做验证时，"发送失败/BusOff"不一定是 MCU 的问题。**

---

## 9. 术语速查表

| 术语 | 一句话解释 |
|---|---|
| Blocking / Polling | CPU 原地反复检查，等不到不干活 |
| IT / ISR | 硬件完成后打断 CPU，执行一小段处理函数 |
| DMA | 专门的搬运硬件，CPU 不参与数据搬移 |
| Message RAM | FDCAN 内部专用 RAM，存放待发/已收帧 |
| TX FIFO / Queue / Buffer | 硬件发送缓冲的三种组织方式（顺序/优先级/专用） |
| TXBAR / TXBTIE / TCE | 发送请求位 / 发送完成中断使能 / 发送完成标志 |
| RxFIFO0 / RxFIFO1 | 硬件接收队列 |
| Filter / Global Filter | 决定哪些帧会被接收进 FIFO |
| Watermark | FIFO 填充到阈值就触发中断（我们关掉了） |
| ACK | 接收方在帧尾给出的确认位；总线上没人收就没有 ACK |
| TEC / REC | 发送/接收错误计数 |
| Error Active / Passive / BusOff | CAN 节点的三个错误状态 |
| AutoRetransmission | 失败自动重发的开关 |
| 非阻塞（non-blocking） | 调用立即返回，用返回值/计数表达"没成功" |

---

## 10. 本工程的改动对应关系

| 改动 | 位置 | 作用 |
|---|---|---|
| `CANTransmit()` 改非阻塞 + 丢帧计数 | `bsp/can/bsp_can.c` | 去掉每个 tick 2~3ms 的自旋 |
| TX FIFO 4 → 16 | `Core/Src/fdcan.c` | 突发时少丢帧 |
| `TxFifoEmpty / TxBufferComplete` 回调 | `bsp/can/bsp_can.c` | 统计"真正发出去多少帧" |
| 接收中断源精简为 `NEW_MESSAGE + MESSAGE_LOST` | `bsp/can/bsp_can.c` | 去掉重复进中断 |
| `rx_lookup[bus][id & 0x1FF]` 查表派发 | `bsp/can/bsp_can.c` | ISR 里从 O(N) 变 O(1) |
| ISR 排空 FIFO（不再提前 return） | `bsp/can/bsp_can.c` | 修掉"FIFO 明明有帧却不处理"的丢帧根因 |
| `CANHealthMonitor()` 10Hz BusOff 恢复 | `bsp/can` + daemon 任务 | 总线不会永久死掉 |
| `CANSetDLC()` 死循环改成钳位+计数 | `bsp/can/bsp_can.c` | 参数错误不再把系统卡死 |
