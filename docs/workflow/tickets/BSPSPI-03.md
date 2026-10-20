# BSPSPI-03：SPI 占用、完成与协作式超时

本文保存实施依据和本机证据；任务状态仅由[TODO](../../../.Doc/TODO.md)维护。本轮没有连接开发板。

## 基线与接口

基线为`fd0ef5c7905adcd2879200db05f3e6ecd5bb7e2b`，共用分支`fix/spi-transfer-ownership`。

原来的发送、接收入口直接拉低片选，忽略HAL返回值；全双工入口另有总线使用者和等待检查。回调按片选电平扫描设备，纯发送也进入接收回调。注册数组只有2项，上限却按4项检查。现在三个入口共用一个事务实现，注册表统一为两条总线合计4项。

- `SPITransmit()`、`SPIRecv()`返回HAL状态；`SPITransRecv()`保留返回方式。阻塞等总线最多1000μs，三类操作的HAL超时均为2ms；IT／DMA遇忙立即拒绝。
- `SPIRegister()`接收只读配置；检查单GPIO片选位、重复片选、句柄归属、模式、期限和分配。每次成功仅申请一个对象；注册／注销由启动流程串行执行。
- `SPISetMode()`返回状态，忙时不改；`SPIUnregister()`遇到本总线活动、回调、恢复或隔离时不释放对象。
- 新增`SPIGetTransferState()`，复制阶段、最近结果与`buffers_released`。拒绝请求不动旧状态及在途字段；异步失败返回也不代表数组可释放。
- 新增`async_timeout_ms`，零使用20ms，显式值1～INT32_MAX ms；配置注册时复制，之后固定。
- 新增`SPIProcessTimeouts()`，接入Daemon约10ms任务循环，使用HAL毫秒时钟。`elapsed >= timeout`判超时；异步期间保留HAL tick lease，防止计时被暂停。

公开用法集中在[bsp_spi.h](../../../bsp/spi/bsp_spi.h)。未增加队列、运行期分配、异步采样或板级IRQ／DMA配置。CMake和旧Makefile既有清单均已包含生产源码，不需要新增源文件。

## 归属、停止与回调

每条总线保存使用者、操作类型和提交／活动／回调／恢复阶段。PRIMASK短临界区只处理状态、GPIO和标志，不包围HAL等待或用户回调。HAL尚未返回就到达的完成先保存，提交返回时在开放任务切换前认领，避免旧完成被用于下一笔事务。

成功接收／全双工才调用接收回调，纯发送不调用；片选先拉高，占用延续到业务回调返回。错误中断仅登记失败。任务调用仓库现有`HAL_SPI_Abort()`，确认HAL READY、SPI使能／启动／请求／相关中断位清除、本次DMA stream停止后，再清理对应标志和NVIC待处理中断并释放。仅有HAL_OK或READY不足以释放。

停止失败时总线与异步数组保持隔离，计时lease继续保留；后续超时检查不重复停止，其他总线仍可运行。恢复需系统重新初始化。阻塞模式未启动DMA／收发中断，返回后其栈数组可以释放；失败接收字节不可使用。检查间隔、调度及HAL停止耗时不属于精确时间保证。

硬件依据为仓库内[HAL SPI源码](../../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c)的`HAL_SPI_Abort()`、`SPI_AbortTransfer()`、`SPI_CloseTransfer()`和正常／错误回调路径。HAL通知没有事务编号；空闲、类型不符或恢复期通知被忽略，同类型任意伪造旧通知仍无法仅凭回调参数识别。安全复用依赖旧硬件停止与清理完成。

## 全部调用点核对

| 调用点 | 当前使用方式 | 本轮处理 |
|---|---|---|
| `modules/BMI088/bmi088.c` | SPI2，阻塞全双工，初始化读写及周期读取 | 保留公式与时序配置；已有调用先检查HAL_OK再复制；触发模式仍拒绝 |
| `modules/imu/ins_task.c` | 配置两个SPI2片选，均为阻塞 | 默认新增期限字段为零，不启用异步 |
| `modules/lcd/lcd.c`、`lcd_buf.c`、`lcd_ui.c` | SPI1直接HAL收发／停止 | 不迁移；运行时不允许同总线另混用本封装 |
| `modules/imu/BMI088Middleware.c` | 旧直接HAL字节入口，无生产调用 | 保留厂商兼容文件，不启用 |
| `bsp/ws2812/ws2812.c` | 其他SPI外设，历史可选路径 | 不迁移，不验证灯珠 |
| 其他单向提交／改模式调用 | 无生产调用，只有新增测试 | 统一接口，不扩展业务调用 |

异步入口检查SPI NVIC、所需DMA stream／Parent／方向／普通模式／字节对齐／内存递增及DMA NVIC，缺少配置直接拒绝。检查不能证明引脚、IRQ处理函数、缓存一致性或实际电平正确；这些不属于本轮软件验收。

## 本机回归证据（2026-10-20）

[生产接口测试](../../../tests/host/bsp_spi_status_test.c)只替换HAL、寄存器、时间和分配器，不增加生产注册表重置接口。目标私有编译定义将生产源码的malloc/free改为计数入口，测试自身仍使用真实标准库。注册失败不分配或不占槽；成功每项一个对象，正常注销分配／释放平衡；运行中分配计数不增加。

| 验证 | 结果 |
|---|---|
| WSL全量Debug／Release | 各200/200通过，包含BMI088、HAL tick lease及既有安全回归 |
| Windows支持子集Debug／Release | 各136/136通过；沿用原排除的`production_init_resource_failures`，未扩大排除范围 |
| SPI专项 | Windows 13项，WSL额外真实线程共14项；每项独立进程，超时10秒，Release检查始终执行 |
| ASan／UBSan | 14/14通过，开启泄漏检查、遇错退出 |
| 最终文件自查 | 补充后注册设备取得占用及错误释放用例后，两平台Debug／Release专项、WSL Sanitizer重新通过；头文件说明整理后六种固件重新构建，尺寸不变 |
| 固件矩阵 | bench_safe、robot、rcdiag × Debug／RelWithDebInfo，六种均禁热并通过 |
| ELF、编译命令 | 配置与执行器隔离检查通过；保留M7／fpv5-d16／hard-float，无fast-math及Ofast，自有源码保留格式告警 |
| 非法配置 | bench_safe开启加热、robot开启遥控诊断均按预期拒绝 |
| 旧Makefile | `make -n -B build/bsp_spi.o`核对现有生产源码和严格格式选项；DSP对象不附加自有告警策略 |
| CMake增量 | 新增／删除／移动源码、多层目录、头目录移动及排除模板均与全新配置一致，基线162个源码路径 |

本轮新增／修改的本地链接及标题锚点、TODO格式和差异空白检查通过。对相关Markdown全文件扫描共检查226个链接，另发现TODO中4个未改动的历史锚点已与目标标题不符，本轮保留；详情存于Windows`build/spi-ownership/doc-links.json`，未将它们计作本轮链接通过。

专项用例覆盖三类操作×三种模式×四个HAL返回值；同总线互斥、不同总线独立；4项容量与第5项拒绝；启动阶段HAL已启用部分硬件才失败的18种组合；20ms的19／20／21ms、显式1ms、起始时刻0及UINT32_MAX回绕；查询不推进期限；错误和恢复期间迟到回调；停止返回错误，以及HAL_OK／SPI READY但DMA仍启用的隔离。

线程用例100轮竞争同一总线，再在19／20ms分别交错完成与超时检查，检查只有一个提交被接受、只有本次结果被发布、lease最终平衡。完成提前于HAL返回的用例检查业务只调用一次；回调中查询仍显示数组被占用，重入提交、改模式和注销均拒绝。内存模型不能证明硬件实际并发时序。

主机复现入口为`cmake -S tests/host -B <目录> -G Ninja -DCMAKE_BUILD_TYPE=Debug`，随后构建并运行CTest；Release使用独立目录。专项使用`ctest --test-dir <目录> -R production_spi_ --output-on-failure`。Sanitizer沿用`-fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie`、链接`-no-pie`，仅构建`bsp_spi_status_test`，开启`ASAN_OPTIONS=detect_leaks=1:halt_on_error=1`与`UBSAN_OPTIONS=halt_on_error=1`。

## 固件尺寸与证据位置

基线和本轮使用相同Linux源码路径与ARM GCC 10.3.1构建；Flash为text+data，静态RAM为data+bss。按输出取值：

| 配置 | 基线Flash | 本轮Flash | 变化 | 基线RAM | 本轮RAM |
|---|---:|---:|---:|---:|---:|
| bench_safe Debug | 174564 B | 185936 B | +11372 B | 104316 B | 104332 B |
| bench_safe RelWithDebInfo | 172256 B | 180228 B | +7972 B | 104268 B | 104284 B |
| robot Debug | 204088 B | 215468 B | +11380 B | 113216 B | 113232 B |
| robot RelWithDebInfo | 199884 B | 207864 B | +7980 B | 113160 B | 113176 B |
| rcdiag Debug | 176744 B | 188124 B | +11380 B | 107484 B | 107500 B |
| rcdiag RelWithDebInfo | 174344 B | 182316 B | +7972 B | 107436 B | 107452 B |

静态RAM各增加16 B；实例内新增字段还会占用既有堆中的空间，表格不将它计作新增静态RAM。Flash增长包含新状态处理、IRQ／DMA检查和实际链接的HAL停止路径。

构建技能脚本因缺少`tool_config`在导入阶段失败，随后使用现有CMake／Ninja入口，未修改技能安装。本轮主机与固件日志位于WSL`/home/mc02/build/spi-ownership/`，Windows`build/spi-ownership/windows-Debug/`、`windows-Release/`；隔离源码为`/home/mc02/build/spi-ownership-source/`，构建相关输入逐文件SHA-256与交付目录对照。增量证据目录为`/home/mc02/build/spi-ownership/regeneration/run-20261020-144325-34sfdifx`。

## 教学与边界

[学习笔记03第13节](../../学习笔记/03-非阻塞通信与CAN收发技术细节.md#13-spi-超时后为什么不能立即交给下一个设备bspspi-03)说明总线、回调和数组期限；[模块说明](../../../bsp/spi/bsp_spi.md)列出状态及调用条件。

仅本机验证；没有启用SPI IRQ／DMA、异步BMI088或连接硬件。SPI时序、DMA物理停止、缓存一致性和吞吐仍需对应硬件任务验证，BSPMISC-07及R09／R27继续保留原范围。复盘：总线空闲与数组可复用必须在确认硬件结束后一起发布，超时标志和HAL READY都不能单独代替停止确认。
