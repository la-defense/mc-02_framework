# 08 内部 Flash 参数存储与 A/B 提交（写给零基础）

> 本篇回答的问题：
> 1. "把数据存到 Flash 里掉电不丢"这句话，在 STM32H7 上具体是什么意思？
> 2. 为什么擦一下要几百毫秒？为什么擦写的时候看门狗要临时放长？
> 3. 为什么要有 A/B 两份？坏了一份怎么自动回退？
> 4. 为什么最终选了 TLV + CRC32 + 序号，而不是直接 `struct` 写进去？
>
> 对应代码：`bsp/param/bsp_param.c`、`bsp/param/bsp_param.h`、`STM32H723VGTx_FLASH.ld`。
> 看门狗那部分与本目录《07-看门狗与复位系统入门》互为补充。

---

## 1. 先搞清楚：为什么不能把参数放在 RAM 里

RAM 里的变量（`.data`/`.bss`）有个特点：**一掉电就没了**。
我们有几类数据特别希望"关机也在"：

| 数据 | 例子 | 丢了会怎样 |
|---|---|---|
| 标定结果 | IMU 陀螺零偏 `gyro_offset[3]`、重力模长 `gNorm` | 每次上电都得重新标定（慢，而且比赛现场不一定有条件） |
| 机器校准 | 底盘轮径、减速比、拨弹盘槽位 | 需要重新手调 |
| 里程/统计 | 累计发射数（本项目不持久化，故意不写 Flash） | 无所谓 |

两条路可以存：

1. **外部 EEPROM / Flash 芯片**（如 AT24Cxx、W25Qxx）：容量大，但要占引脚和 PCB 面积，还要写驱动；
2. **单片机自己的内部 Flash**：不用加任何硬件，代价是"它本来是放程序的"，
   擦写有各种限制——本篇讲的就是这条路。

---

## 2. STM32H723 的内部 Flash 长什么样

先记住四个事实（后面所有结论都从它们推出来）：

### 2.1 它是"扇区"结构，不是"字节"结构

STM32H723VG 有 **1MB 内部 Flash**，被切成 **8 个扇区，每扇区 128KB**，而且**只有 1 个 bank**（单 bank）：

| 扇区 | 起始地址 | 大小 |
|---|---|---|
| 0 | `0x0800 0000` | 128KB |
| 1 | `0x0802 0000` | 128KB |
| ... | ... | ... |
| 6 | `0x080C 0000` | 128KB |
| 7 | `0x080E 0000` | 128KB |

**擦除的最小单位是"扇区"**（128KB 一次），**写入的最小单位是"Flash Word"= 32 字节**。
你不能"只改 4 个字节"：

- 想把某 4 个字节从 `0x11` 改成 `0x22` → 需要先擦掉整个 128KB，再把内容重新写回去；
- 擦一个扇区在 H7 上要**几百毫秒到几秒**（实测本项目擦 1 个扇区约 0.8s，擦 2 个约 1.7s）。

### 2.2 位操作是"只能把 1 变 0"

Flash 的物理特性决定了：

- **擦除** = 把整片区域变成全 `1`（`0xFF`）；
- **写入** = 把某些 `1` 变成 `0`；
- **已经变成 0 的位，不能再变回 1**（除非再擦一次）。

所以"改一个参数"的标准流程永远是：**擦掉 → 写新的**。

### 2.3 擦写时 CPU 会停摆（单 bank 的代价）

双 bank 的芯片可以"在 bank2 里写，同时从 bank1 里取指令"。H723 是单 bank，
所以在写/擦 Flash 的时候，**CPU 从 Flash 取指会被硬件暂停**，等擦写完成才继续。

这意味着两件事：

1. 擦写期间**所有任务都停了**（不是"变慢"，是"完全不动"）；
2. 擦写期间**没人喂狗** → 必须提前把看门狗超时放长（见第 6 节）。

### 2.4 还有一个容易踩的坑：Cache

Cortex-M7 有数据缓存（D-Cache）。如果开了 D-Cache，CPU 读的是**缓存里的旧内容**，
而你刚写进 Flash 的新内容还在缓存之外 → 回读校验会读到"旧数据"，误判失败。

本工程目前没有显式打开 D-Cache（`Core/Src/main.c` 里没有 `SCB_EnableDCache()`），
但代码里仍然老老实实加了 `SCB_InvalidateDCache_by_Addr()`：
将来谁开了 Cache 也不会踩坑。这是一个很便宜的"防御式编程"。

---

## 3. 布局：为什么把最后两个扇区留给参数

程序本体编译出来只有 ~150KB（1024KB 里的 15%），根本用不完。
所以直接把**最后两个扇区（6/7）从程序区里切出去**，专门放参数：

```ld
/* STM32H723VGTx_FLASH.ld */
FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 768K   /* 原来是 1024K */
```

做完这一步，编译器和链接器就再也不会往 `0x080C0000` 之后放代码了——
否则可能出现"擦参数把程序擦掉一半"的惨案。

**记住这条铁律：只要你在用某块 Flash 存数据，就必须在链接脚本里把它从程序区里挖掉。**

---

## 4. 记录格式设计：magic + 版本 + 长度 + 序号 + CRC + TLV

### 4.1 头部（32 字节，正好等于一个 Flash Word）

```c
typedef struct {
    uint32_t magic;         /* 'MC02'，用来判断"这块 Flash 到底是不是我们的参数区" */
    uint16_t version;       /* 格式版本，未来改格式时用 */
    uint16_t payload_len;   /* 后面 payload 有多少字节 */
    uint32_t seq;           /* 序号，每提交一次 +1，用来判断"哪个区更新" */
    uint32_t crc32;         /* payload 的 CRC32，用来判断"内容有没有坏" */
    uint32_t reserved[4];   /* 补齐到 32 字节(Flash Word 对齐) */
} ParamHeader_t;
_Static_assert(sizeof(ParamHeader_t) == 32, "记录头必须正好 32 字节");
```

为什么要对齐到 32 字节？因为 H7 的写入最小单位就是 32 字节（Flash Word）。
头部正好占满一个 Flash Word，写起来最省事。

### 4.2 为什么要有 `magic`

上电时我们面对的是"一整片可能全是 0xFF、也可能是别的固件留下的垃圾"的 Flash。
靠 `magic` 一眼就能判断"这里到底有没有我们的记录"，比"猜长度合不合理"可靠得多。

### 4.3 为什么用 `seq` 而不是时间戳

两个区都有效时，要回答"哪个是最新的"。可选方案：

| 方案 | 问题 |
|---|---|
| 存 RTC 时间戳 | 本板没有电池供电的 RTC，上电时间从 0 开始，不可比 |
| 存"写入时刻（uptime 秒）" | 重启后就归零，也不可比 |
| **存序号 seq，每次提交 +1** | 单调递增，跨重启依然可比 ✓ |

所以 `seq` 是这类"主备存储"里最常用的方案（跟你手机上的 A/B 系统更新是一个思路）。

### 4.4 为什么用 CRC32 而不是"校验和"

Flash 坏了通常是**整位翻转**（比如某一位从 0 变 1）。
简单求和（checksum）对"两个字节互相抵消"的错误没辙；CRC32 能检出绝大多数常见错误形态。
本项目用**软件实现**的 CRC32（IEEE 多项式 `0xEDB88320`，和 zip/png 一样），
因为 HAL 的硬件 CRC 外设当前没使能；参数提交是低频操作，软件算完全够用。

**但要记住 CRC 的能力边界**：它只能证明"字节没被改坏"，
**证不了"数值合理"**。所以 `bmi088.c` 里读出来之后还会再判一次
"零偏是不是在 ±0.1 以内、重力模长是不是在 9.0~10.5 之间"——
这叫**语义校验**，和 CRC 的**完整性校验**是两层防护，缺一不可。

### 4.5 payload 为什么用 TLV

TLV = **T**ype（键）+ **L**ength（长度）+ **V**alue（数据）：

```
[key:2B][len:2B][data: len 字节][补齐到 4 字节] [key:2B][len:2B][data...] ...
```

好处：

1. **加字段不影响老记录**：新固件多写一个 key，老记录里没这个 key，
   `ParamGet()` 返回 0，调用方用默认值——不需要"格式版本升级"这种重活；
2. **长度自适应**：`float`、`float[3]`、`struct` 都能塞进去；
3. **查找便宜**：顺序扫一遍 512 字节的 RAM 缓存就完事，不需要建立索引。

本工程的 key 分配：

| key | 含义 | 长度 |
|---|---|---|
| `0x0100` | `PARAM_KEY_IMU_GYRO_OFFSET` | 3×float（12B） |
| `0x0101` | `PARAM_KEY_IMU_G_NORM` | float（4B） |
| `0x0102` | `PARAM_KEY_IMU_CALIB_META` | 结果 + 温度 + 时刻（12B） |
| `0x0200~` | 预留：CAN 参数 | — |
| `0x0300~` | 预留：底盘参数 | — |
| `0x0400~` | 预留：配置框架 | — |

高字节分段是为了将来"一眼看出这个 key 属于哪个模块"。

`PARAM_KEY_IMU_CALIB_META` 里的温度字段保存的是**摄氏度**，不是 BMI088 寄存器原始码。按 [Bosch BMI088 数据手册 §5.3.7](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)，温度由 11 位二进制补码组成：先拼出 `raw = (temp_msb << 3) | (temp_lsb >> 5)`；若 `raw > 1023`，要减去 `2048` 做符号扩展；最后按 `23 + signed_raw × 0.125°C` 换算。比如原始码 `2047` 表示 `-1`，温度是 `22.875°C`。

工程里的 [`BMI088DecodeTemperature`](../../modules/BMI088/bmi088_temperature.h) 在采样入口完成这次换算；在线标定拿到的 `raw_data.temperature` 已经是摄氏度，所以直接保存。再乘 `0.125` 并加 `23` 会把同一个值换算第二次。边界回归在 [`bmi088_temperature_test.c`](../../tests/host/bmi088_temperature_test.c)：覆盖原始码 `0、1023、1024、2047`，以及数据手册给出的 `-40°C`／`85°C` 端点。

---

## 5. A/B 双区：为什么"坏一份还能活"

### 5.1 直接原地覆盖的问题

假设只有一个区：擦除（1.7s）→ 写新值（1ms）。
**如果"擦完还没写"的时候断电了**，这个区就空了，参数全丢——
而且恰恰是"最需要参数"的场合（比如比赛现场被人踢了电源）。

### 5.2 A/B 交替怎么写

```
当前活动区 = B (seq=3)

提交新参数时:
  1. 擦掉"非活动区" A          ← 活动区 B 完好无损
  2. 把新记录(seq=4)写进 A
  3. 回读校验 A, 确认写对了
  4. 活动区切到 A

断电发生在 1/2/3 任何一步 → 活动区 B 里的 seq=3 依然完整可用
```

上电时 `ParamInit()` 的规则很简单：

```
读 A 的头 → magic/version/长度/CRC 全对 → valid_a = 1
读 B 的头 → 同上                      → valid_b = 1

两个都有效 → 取 seq 大的那个
只有一个有效 → 用有效的那个（这就是"坏一份自动回退"）
两个都无效  → 参数为空，调用方用默认值（并报警）
```

### 5.3 为什么提交后要"回读比对"

写完不等于写对（电压不稳、Flash 老化、写保护都可能让某个字写失败）。
所以 `ParamWriteSector()` 最后一步是：

```c
SCB_InvalidateDCache_by_Addr((uint32_t *)sector_addr, total);
if (memcmp((const void *)sector_addr, buf, total) != 0) {
    LOGERROR("[param] 回读校验失败");
    return 0;   /* 活动区不切换, 继续用旧的那份 */
}
```

只要回读失败，就**不切换活动区**——宁可继续用旧参数，也不用一份没验过的。

---

## 6. 提交期间的看门狗窗口（重要）

这一节和《07-看门狗与复位系统入门》配套看。

### 6.1 冲突从哪来

- 运行期 IWDG = **200ms**，由 `TaskMonitorTick()` 在 daemon 任务里喂；
- 擦一个扇区最坏要 **~1.7s**（两块一起擦实测 1.66s）；
- 擦写期间 CPU 停摆 → daemon 也停 → **200ms 的看门狗必然先超时复位**。

### 6.2 处理办法：临时放长，做完恢复

```c
uint32_t prev_timeout = BSP_WatchdogGetTimeout();
BSP_WatchdogSetTimeout(8000);   /* 8s > 最坏擦写时间(实测 ~1.7s, 留 4 倍余量) */
BSP_WatchdogFeed();

... 擦除 + 写入 + 回读 ...

BSP_WatchdogSetTimeout(prev_timeout);  /* 恢复 200ms(或启动期的 4s) */
BSP_WatchdogFeed();
```

三个细节：

1. **必须保存并恢复原值**，不能写死 200ms——因为启动期看门狗是 4s，
   首次自动标定保存参数时如果恢复成 200ms，启动流程会立刻被复位；
2. 放长之后**马上喂一次**，避免"刚放长又超时"；
3. IWDG 的 `PR/RLR` 寄存器有更新等待位（`PVU/RVU`），写之前要等它们清零，
   否则新值可能没生效——本工程 `bsp_watchdog.c` 里有 `IWDG_WaitForUpdateFlags()` 处理。

### 6.3 还可以再严格一点的做法（本项目没做，留作后续）

- 提前"预擦"非活动区（在 SAFE 状态、电机失能时后台擦），真正提交时只剩写入（~1ms），
  这样连放长看门狗都不需要；
- 或者把参数放到外部 Flash / FRAM，彻底避开"擦写停 CPU"。

---

## 7. 代码走读（按调用顺序）

```
BSPInit()                        ← 启动早期（DWT→看门狗 4s→日志→ADC→复位原因）
  └─ ParamInit()                 ← 扫 A/B、校验、把更新的那份读进 RAM 缓存

GimbalInit/ChassisInit → INS_Init → BMI088Register
  └─ BMI088CalibInit()
       ├─ ParamInit()            ← 幂等：已经扫过就直接返回，不会覆盖 RAM 缓存
       ├─ BMI088CalibLoad()      ← ParamGetFloats/ParamGetFloat(+语义校验)
       │    成功 → 用 Flash 值, RobotSafetySetCalibValid(1)
       └─ 失败 → 首次自动标定 → BMI088CalibSave()
                                  └─ ParamSet×3 → ParamCommit()

运行期（LCD 长按 3s）
  └─ BMI088CalibRequest() → INS_Task 里 BMI088CalibService()
       └─ 标定成功 → BMI088CalibSave() → ParamCommit()
```

### 7.1 `ParamSet()` 的两种路径

```c
/* 已经存在同长度的条目 → 原地覆盖, 最简单 */
if (data_off != 0 && entry_len == len) { memcpy(...); s_dirty = 1; return 1; }

/* 新 key 或长度变了 → 先把旧条目从 RAM 缓存里"抠掉", 再追加到末尾 */
memmove(...);   /* 把后面的条目往前挪 */
... 追加 [key][len][data][pad] ...
```

注意这里操作的是 **RAM 缓存**，Flash 一个字节都没动——真正落盘只在 `ParamCommit()`。
这个设计让"连续设 3 个 key 再提交一次"变成一次擦写，而不是三次。

### 7.2 为什么 `ParamInit()` 要做成幂等

如果把 `ParamInit()` 写成"每次调用都重新扫 Flash 并覆盖 RAM 缓存"，
那么"先 `ParamSet()` 改了值、还没来得及 `ParamCommit()`，另一个模块又调用 `ParamInit()`"
就会把刚改的值冲掉。加了 `static uint8_t s_inited` 之后，第二次调用直接返回上次结果。

---

## 8. 实测验证方法（你可以照着复现）

用 OpenOCD 读几个全局变量就能看到全流程（地址用 `arm-none-eabi-nm` 查）：

| 变量 | 含义 |
|---|---|
| `param_loaded` | 1 = 启动时从 Flash 成功加载 |
| `param_active_region` | 0 = A 区，1 = B 区 |
| `param_seq` | 当前记录的序号 |
| `param_commit_count` / `param_commit_fail` | 提交成功 / 失败次数 |
| `bmi088_calib_source` | 0=未定 1=FLASH 2=首次自动 3=手动 4=默认值 |

**测试 1：首次上电（两个区都是 0xFF）**

```bash
# 擦掉参数区
openocd ... -c "init" -c "halt" -c "flash erase_sector 0 6 7" -c "reset" -c "exit"
```

预期日志：`[bmi088] 无 Flash 标定记录, 首次自动标定...` → `标定成功并已保存`
预期变量：`param_seq=1`、`param_active_region=1`（第一次写 B 区）、`cali_diag_outer>=1`

**测试 2：再次复位（有记录）**

预期日志：`[bmi088] 使用 Flash 标定记录`
预期变量：`param_seq` 不变、`cali_diag_outer == 0`（**一次标定循环都没跑**，启动 <1s）

**测试 3：连续提交，看 A/B 交替**

预期：`param_seq` 1→2→3，`param_active_region` 1→0→1（严格交替）。

**测试 4：把活动区擦掉（模拟"坏了一份"）**

```bash
openocd ... -c "init" -c "halt" -c "flash erase_sector 0 7 7" -c "reset" -c "exit"
```

预期：自动回退到另一个区，`param_seq` 回到备份区的值（实测 3→2、区域 B→A），
而且**不会重新标定**。

**测试 5：两个区都坏**

预期：回到"参数为空"分支 → 首次自动标定 → 保存 → 报警（如果标定失败则置 `CALIB_INVALID`）。

---

## 9. 常见坑清单

| 坑 | 后果 | 正确做法 |
|---|---|---|
| 忘了改链接脚本 | 擦参数把程序擦掉 | 用 Flash 存数据前先把这段地址从 `FLASH` 长度里减掉 |
| 用 `UINT32_MAX` 当 2^32 的模 | 每次回绕少算 1 | 增量累加用 32bit 无符号减法（见《09》） |
| 在中断里提交参数 | ISR 里擦 1.7s Flash，系统必崩 | 只在任务上下文、SAFE/CALIB 状态提交 |
| 提交时不放长看门狗 | 200ms 必然复位，还会形成复位循环 | `SetTimeout(8000)` → 擦写 → 恢复原值 |
| 恢复成写死的 200ms | 启动期（4s）被缩短，启动立刻复位 | 先 `GetTimeout()` 存下来再恢复 |
| 只做 CRC 不做语义校验 | 参数区被别的固件写过时，CRC 可能"合法"但值荒谬 | CRC + 范围检查两层 |
| 开了 D-Cache 不注意一致性 | 回读校验读到缓存里的旧值 | 读前 `SCB_InvalidateDCache_by_Addr` |
| 把 `ParamSet()` 当成落盘 | 掉电后什么都没存 | `ParamSet` 只改 RAM，必须 `ParamCommit()` |

---

## 10. 一句话总结

**内部 Flash 存参数 = 用"擦得慢、写得粗、单 bank 还要停 CPU"的介质，
换"不增加任何硬件就能掉电保持"；代价用 A/B 双区 + CRC + 序号 + 临时放长看门狗来兜。**
