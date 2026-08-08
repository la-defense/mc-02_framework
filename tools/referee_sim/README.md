# 裁判系统信号模拟器（测试方法记录）

> 本文档记录 2026-08-08 使用 USB-TTL 模拟裁判系统对 MC-02 下位机做端到端测试的方法与结论，
> 作为将来开发完整“裁判系统模拟器”软件的输入与铺垫。

## 1. 目的

在没有真实裁判系统硬件的情况下，用 PC + USB-TTL 模块按官方协议向机器人主控投喂裁判数据，
验证下位机裁判协议解析、离线守护、异常帧防护等逻辑。

## 2. 硬件接线

- 模块：CH340 USB-TTL（本机 COM4，需 `pyserial`）
- 接线（关键，务必核对）：
  - CH340 **TX → MCU PA10（USART1_RX）**
  - CH340 **RX → MCU PA9（USART1_TX）**（只测试接收时可不接）
  - **GND → GND（必须共地）**
- 电平：开发板 I/O 为 3.3V，USB-TTL 请使用 3.3V 电平，勿用 5V TX 直连。
- 波特率：**115200**，8N1（与固件 USART1 一致；协议文档中“常规裁判链路”为 115200，
  图传链路为 921600）。

## 3. 协议帧格式（RoboMaster 2026 通信协议 V2.0.0）

```
frame_header(5B) | cmd_id(2B) | data(nB) | frame_tail(2B, CRC16)
frame_header: SOF(0xA5) + data_length(2B, 小端) + seq(1B) + CRC8(1B)
```

- CRC8：官方表（init 0xFF），见脚本 `CRC8_TAB` / 固件 `crc_ref.c`
- CRC16：init 0xFFFF、poly 0x8408（reflected），与脚本 `crc16()` 一致

## 4. 使用方法

```bash
pip install pyserial
python referee_sim.py valid    # 正常帧: 0x0001/0x0201/0x0202/0x0207 连续投喂
python referee_sim.py large    # 超大 data_length=0xFFFF(合法CRC) -> 触发长度边界保护
python referee_sim.py short    # 0x0001 但 data_length=0 -> 触发数据段长度保护
python referee_sim.py burst    # 200B 0xA5 洪泛 -> 防递归/防崩溃
python referee_sim.py multi    # 双帧拼接一次发送 -> 验证循环解析
```

默认串口/波特率在脚本顶部 `PORT`/`BAUD` 修改。

## 5. 验证方法（OpenOCD + GDB）

```bash
# 启动后台 OpenOCD（不暂停目标）
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c init

# 读取解析结果（固件符号: referee_info, referee_daemon, huart1）
arm-none-eabi-gdb -q -batch build/Basic_Framework_MC02.elf \
  -ex "target extended-remote localhost:3333" -ex "monitor halt" \
  -ex "p referee_info.GameRobotState" \
  -ex "p referee_info.PowerHeatData" \
  -ex "p referee_info.ShootData" \
  -ex "p referee_info.GameState" \
  -ex "p referee_info.CmdID" \
  -ex "p *referee_daemon" \
  -ex "monitor resume" -ex "detach" -ex "quit"
```

异常帧后检查是否崩溃：

```gdb
p/x $pc
p/x *(uint32_t*)0xE000ED34   # CFSR, 应为 0
```

## 6. 2026-08-08 实测结论

- 合法帧 0x0001/0x0201/0x0202/0x0207 全部正确解析：
  `robot_id=7`、`chassis_power_limit=45`、`bullet_speed=30`、`game_progress=4`；
- 超大 data_length、0xA5 洪泛、data_length=0 短帧、双帧拼接均不崩溃（CFSR=0、PC 正常）；
- data_length=0 的短帧未覆盖目标结构体（按命令码的数据段长度保护生效）；
- 双帧拼接在一次接收中全部解析（循环解析生效）；
- 裁判 daemon 在停帧约 300ms（reload_count=30 @100Hz）后离线，恢复投喂后重新上线。

## 7. 未来模拟器软件建议（待开发）

- 图形界面（PySide/Qt 或 Web）：按命令码编辑数据段、实时显示已投喂帧；
- 按真实频率/时序自动周期发送（如 0x0201/0x0202 约 50Hz，0x0001 约 10Hz）；
- 错误注入：坏 CRC、超大长度、乱序/丢帧、0xA5 洪泛、多帧拼接、半帧截断；
- 读取下位机回传（USART1 TX → CH340 RX）用于验证 UI/交互上行；
- 支持不同机器人 ID/颜色切换、比赛阶段/时间倒计时等场景化配置；
- 记录与回放（抓包保存、回放异常序列复现问题）。
