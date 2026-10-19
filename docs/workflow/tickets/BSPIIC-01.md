# BSPIIC-01：I²C 注册分配与失败回退

## 基线与范围

- 基线：`e9b186fbe165b4dc7aff03fb7c2f41e595f9a3be`；分支：`fix/iic-registration`。
- 本任务修复注册配置检查、重复检测、注册表容量和 TFmini/IST8310 的注册失败回退。本任务不处理 I²C 传输状态、GPIO 生命周期、IST8310 登记后的设备校验失败或 TFmini 接收长度问题。

## 实现记录

- 注册容量统一由 `MX_IIC_SLAVE_CNT` 表示，为8个设备；`IIC_DEVICE_CNT` 仍表示物理接口数。
- `IICRegister(const IIC_Init_Config_s *)` 检查空配置/句柄、7位地址、工作模式、同句柄同地址重复项、容量和分配结果。拒绝重复项，不替换已有对象。
- 只有完整清零并复制配置的实例才会进入注册表；地址在此处左移一次，短临界区只负责发布指针和数量。
- TFmini 和 IST8310 在外层对象或 I²C 注册失败时释放候选对象。IST8310 在注册成功前使用局部配置，失败不写回传入配置，也不进行等待、GPIO 登记或设备访问。

## 验证证据

- WSL Debug/Release 全量主机测试均为 161/161；Windows Debug/Release 支持子集均为 72/72。三个生产注册/调用方场景在两端均通过。
- ASan/UBSan 聚焦回归 33/33 通过；bench_safe、robot、rcdiag × Debug/RelWithDebInfo 六种固件构建及 ELF profile 检查通过，非法加热和非法 rcdiag 配置均按预期拒绝。重复配置并连续构建的 CMake/Ninja 增量检查显示无重建。
- 本任务不连接开发板，不声称已验证 I²C 波形或设备通信。

### 固件容量记录

以 `e9b186f` 基线和本轮固件比较 `arm-none-eabi-size` 的 `text+data`（程序存储）与 `data+bss`（静态RAM）：

| 配置 | 基线 Flash | 本轮 Flash | 变化 | 基线 RAM | 本轮 RAM | 变化 |
|---|---:|---:|---:|---:|---:|---:|
| bench_safe Debug | 168828 B | 168692 B | -136 B | 104628 B | 104628 B | 0 B |
| bench_safe RelWithDebInfo | 168728 B | 168592 B | -136 B | 104696 B | 104696 B | 0 B |
| robot Debug | 197768 B | 197632 B | -136 B | 113504 B | 113504 B | 0 B |
| robot RelWithDebInfo | 196264 B | 196136 B | -128 B | 113544 B | 113544 B | 0 B |
| rcdiag Debug | 171044 B | 170916 B | -128 B | 107804 B | 107804 B | 0 B |
| rcdiag RelWithDebInfo | 170848 B | 170720 B | -128 B | 107864 B | 107864 B | 0 B |

这六种配置没有链接 `IICRegister()`、TFmini 或 IST8310 的应用符号，因此 RAM 未变化；少量 Flash 差异不作为本任务代码节省的结论。

## 教学入口

- [学习笔记03：非阻塞通信与 CAN 收发技术细节](../../学习笔记/03-非阻塞通信与CAN收发技术细节.md)
- [BSP I²C 注册接口](../../../bsp/iic/bsp_iic.h)
