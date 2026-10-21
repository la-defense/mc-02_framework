# REF-03：裁判发送共享预算与裸板回归记录

任务状态只在[TODO](../../../.Doc/TODO.md)维护。本轮基线`5c60d927398226cb9b0b6af391002b41af333e3b`，共用分支`feat/referee-shared-budget`。保留未跟踪资料，不推送、不创建PR。

## 协议与实现依据

唯一基线为本地官方《RoboMaster 2026机甲大师高校系列赛通信协议V2.0.0（20260626）》；PDF SHA-256为`8fdc484c58d503a3fb68f7c0515ae618b68eb2b058a8c876e1add5eeb8a7d81b`。PDF第24～25页（正文页码23～24）规定0x0301合计30Hz、完整帧127字节、内容112字节；附录PDF第80～81页给机器人／客户端身份。修正旧10Hz／113字节注释，不接入服务器自主决策命令。

- [共享公开入口](../../../modules/referee/referee_tx.h)复制提交、取消和只读查询；[组帧](../../../modules/referee/rm_referee.c)检查新鲜身份、同队目标、六类UI准确长度及机器人子ID0x0200～0x02FF，统一真实CRC与序号。删除公开原始帧RefereeSend和可写UI_Seq。成功入队才递增序号；拒绝保持编号、不通知，后来失败／取消不回退序号。
- [纯C额度计算](../../../modules/referee/referee_tx_budget.c)由裁判实例长期保存30条启动记录，检查34ms启动间隔和滚动1000ms目标字节额度。普通机器人3720字节，哨兵／雷达5120字节；年龄恰好1000ms释放旧记录。只统计本发送者，不宣称覆盖其他发送者。身份变化及UI重新初始化不清历史。
- 复用每路四条UART复制存储及原发送任务，不新增帧队列或任务。注册时固定启动检查与完成观察；暂缓保持FIFO队首、原内容及期限，不让后续帧越过。先观察当前终态再启动下条，回调在临界区外执行。已开始失败不退额度，启动报错按完整帧保守计费，确认未启动的取消／过期不计费。
- [UI状态机](../../../modules/referee/referee_task.c)保留原15帧顺序、最多一条未完成请求、115ms实际启动节奏、100ms总期限及版本对应。尚未启动就过期保留步骤／刷新标记，下轮尝试最新内容；HAL启动错误、已开始后的失败／超时与隔离仍停止并等待显式初始化。身份过期或变化停用旧代，接收不回滚。
- 查询统计供诊断：UART和裁判统计分别复制，不构成跨模块原子读取或控制许可。提交并发争用立即返回BUSY，计入拒绝；运行期没有新增分配。

## 主机验收

WSL Ubuntu22.04、GCC11.4.0、ARM GCC10.3.1，CMake3.22.1／Ninja；Windows原生构建不作为验收。现有缓存`/home/mc02/build/uart-tx-{debug,release,asan}`复用，目录名不代表仍是旧源码。

WSL全量Debug／Release各**275/275**通过。适用ASan／UBSan／浮点转整数溢出检查**66/66**通过；相关场景选择`production_uart|production_referee|production_crc|production_task_monitor|production_usart`，编译`-fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer -fno-pie`，链接同组消毒选项和`-no-pie`。新增每场景独立进程、10秒超时，Release使用TEST_CHECK。

```sh
cmake -S tests/host -B /home/mc02/build/uart-tx-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build /home/mc02/build/uart-tx-debug -j 10
ctest --test-dir /home/mc02/build/uart-tx-debug -j 6 --output-on-failure
# Release使用独立目录并指定-DCMAKE_BUILD_TYPE=Release。
```

[referee_budget_test.c](../../../tests/host/referee_budget_test.c)使用生产CRC、裁判解析／身份、UI、UART队列及H723后端；仅HAL、RTOS、时间和Daemon使用模型。12个新场景覆盖参数／长度／身份、4条容量和第5条拒绝、FIFO、pthread并发提交、UI未启动超期、34ms、滚动1000ms及普通／特殊机器人差异、跨目标、序号回绕、HAL启动错误、时间回绕和身份不能清预算、未启动取消／过期不扣费、HAL_BUSY、DMA完成不同于UART完成、停止失败隔离和其他串口独立。运行期包装分配器检查无新增分配／释放；既有UART、UI、CRC、安全回归继续执行。

[专用工具离线回归](../../../tests/tools/test_referee_budget_capture.py)6项覆盖固定CRC向量、全部分割位置、噪声／错误CRC、完整／残帧、错误ELF／记录布局、丢环记录、短写／超时、只读命令范围及报告拒绝。它不连接硬件。CRC独立逐位参考`123456789`：裁判CRC8=0x0B、SP／裁判CRC16=0x6F91；旧UI固定整帧向量仍由生产接口回归。

## ARM构建、隔离与资源

六种常规禁热配置bench_safe／robot／rcdiag × Debug／RelWithDebInfo通过；另有robot关闭UI、备用UART视觉和独立refdiag Debug。具名ELF隔离、非法bench加热／UI、robot遥控或裁判诊断、两种诊断同时打开、双视觉传输选择均按预期拒绝。实际编译命令核对严格格式选项、Cortex-M7／fpv5-d16／hard-float，无fast-math／Ofast，ELF为VFP寄存器传参。UART固定ports仍32字节对齐并在AXI RAM；UI关闭时不保留UI任务与发送入口，接收和UART推进仍存在。

CMake增量检查在`/home/mc02/build/ref03-regeneration/run-20261021-210545-rhcj1i5z`完成15类增删／移动场景，167份基线源码和每步干净配置一致。旧Makefile加入额度计算和诊断源码，未推进BUILD-02。build-cmake技能入口因共享tool_config缺失无法启动，本轮直接使用仓库WSL CMake入口；没有修改外部技能环境。

资源口径为Flash=text+data、RAM=data+bss，同工具链与本轮基线比较。完整数值与ELF校验值见[构建资源](REF-03-evidence/firmware.json)。


| 配置 | Flash字节 | 相对基线 | RAM字节 | 相对基线 |
|---|---:|---:|---:|---:|
| bench_safe Debug | 198308 | +96 | 111100 | +0 |
| bench_safe RelWithDebInfo | 189444 | +128 | 111052 | +0 |
| robot Debug | 229464 | +1164 | 119720 | +440 |
| robot RelWithDebInfo | 218512 | +1164 | 119696 | +440 |
| rcdiag Debug | 200536 | +120 | 114268 | +0 |
| rcdiag RelWithDebInfo | 191564 | +140 | 114220 | +0 |

## 专用裸板回归入口

[referee_bench.c](../../../modules/referee/referee_bench.c)仅MC02_BENCH_REFEREE_DIAGNOSTICS启用；配置只允许独立bench_safe、禁热、无执行器初始化，不请求READY。复用RobotTask，不创建任务。等待第一份新鲜身份后运行30秒满长127字节交互，再运行45秒混合UI空操作／交互，随后结束；普通六配置无自动测试流量。UI空操作内容0100/00 00不要求真实客户端。

[采集工具](../../../tools/referee_budget_capture.py)直接使用3.3V TTL、USART1 115200/8N1；接线为适配器TX→PA10、RX←PA9、共地、VCC悬空。只发送合法0x0201机器人身份，不发送控制、使能或开火命令，不运行旧模拟器GUI／MQTT，不修改独立模拟器仓库。工具80秒有界采集，前30秒满长、后续混合；65秒停更1秒、70秒变更身份。原始UART、CSV、身份注入记录及只读MCU环形完成记录一起保存；MCU记录用于预算，PC时间只作交叉观察。错误CRC、残帧、丢记录、布局／身份不匹配和时长不足都不能报告通过。

工具要求session JSON的schema=1与firmware字段：source_sha、elf_sha256、flash_verified=true；核对给定ELF，不代替实际Flash verify。运行期间通过固定白名单只读RAM，MCU保持运行；64条记录必须及时采集，发生丢环就中止。记录全部显式32位字段，编译期检查1824字节布局，避免ARM／主机枚举尺寸影响解析。

## 上板证据登记

用户确认12.5V／2A、LCD＋DR16、板供DAPLink、无执行器，3.3V TTL按PA10／PA9交叉连接且VCC悬空。本机枚举COM3=CH340、COM9=DAPLink、COM14=MC02 USB。首次只读备份在CMSIS-DAP CMD_INFO阶段失败，未停核／复位／擦写；用户随后重新插拔DAPLink。重新连接后先完成第一份1MiB读取（109.5秒），初始120秒总时限中断第二份；加长时限后完成第二份与参数读取。两份整片Flash SHA-256均为`2a27ff62c44066a64cf74fbbf9dc8bf94e8c1a1917fd96b9b574c777427e2821`；独立256KiB参数区与整片0xC0000切片逐字节一致，SHA-256为`413f4da1afa9483a6bf156cb7124f05276927eb843ce5d040c85725257120de4`。备份未停核、复位或擦写。读取错序后用4KiB对照复读验证恢复，再完成长读取。软件SHA、verify、烧录后参数保留、60秒基线及有限负载结果将在实际完成后补充。

本轮文档链接检查通过；同时修正TODO中四条历史旧锚点，只调整链接、不改变其余任务状态。差异空白及TODO格式通过。

## 教学与范围

实质更新[学习笔记03](../../学习笔记/03-非阻塞通信与CAN收发技术细节.md)、[学习笔记20](../../学习笔记/20-SP固定帧与CRC校验.md)、[索引](../../学习笔记/README.md)和[裁判模块说明](../../../modules/referee/referee.md)。MC02-0022仍需通用接收／业务接口，MC02-0095仍需车型布局、真实来源与自动重绘。本机和裸板串口均不保证对端接收、客户端显示、精确波形或其他发送者的额度；停止失败的真实DMA物理时序仍需专门台架。

复盘：共享预算放到唯一复制发送入口，调用方不再分别维护序号与CRC；保留原四条存储和任务。验收区分入队、实际启动和本机完成，避免把等待超期误当作客户端显示失败。


## 首次上板发现与启动修复

软件提交`45f9c956df289fdda54688d9cb428114fb1dc1b5`的refdiag ELF SHA-256为`2ad749c58332d37f5417d2923c6589e879f3dc36989a8973fc50cdc950d931cf`。技能报告program／verify成功后，在缓存保存阶段因缺失no_save_profile参数退出；未重复烧录，改用独立verify_image确认204244字节一致。参数回读与烧录前逐字节一致，SHA-256保持不变。

初始被动检查未通过：启动诊断stage=3、module=8、reason=2、resource=8，即UART发送任务创建失败，INS尚未启动。没有发送负载。精确复现采用生产uart_tx_worker_test在PRIMASK=1下创建任务，修复前因没有调用RTOS创建而失败。

修复仅调整USARTTxTaskInit的启动检查：调度器启动前Thread模式允许PRIMASK=1，保持原状态；ISR、BASEPRI或FAULTMASK非零拒绝。运行期发送检查不放宽。新增回归覆盖关中断的失败重试与成功、ISR拒绝、中断状态保持。重新执行软件验收后，以新的准确提交和ELF更新板上固件；最终硬件证据另补。第一次失败的基线记录保留，不能计入60秒通过观察。


启动修复后的WSL Debug／Release仍各275/275，消毒器66/66；六种常规ARM、UI关闭、备用UART及诊断重新构建通过，隔离和编译选项检查通过。二次烧录前完整回读当前失败诊断固件1MiB，SHA-256为`21c4c1bcd7321b194c44dc48867c91119a0e57ad26c23e0a9e574fd7674dd83e`，参数切片与初始备份仍逐字节一致；未先运行任何串口负载。


第二次观察仍定位UART_TX（提交7485322）：FreeRTOS在调度器启动前的uxCriticalNesting=0xAAAAAAAA，前序任务已创建，scheduler=0、UART任务为NULL，剩余RTOS堆8648字节。该版本仍拒绝BASEPRI非零，未覆盖真实端口启动行为。已扩展复现为PRIMASK=1、BASEPRI=0x50，确认修复前失败；最终启动检查只拒绝ISR或FAULTMASK，并在内核未运行时创建，允许RTOS保留的两种普通中断屏蔽。运行期入口没有放宽。另覆盖调度器已运行和FAULTMASK拒绝，继续复验后使用新软件SHA烧录；前两次短失败检查均不作为通过证据。


最终启动检查修复后复验WSL Debug／Release各275/275、消毒器66/66；六种常规ARM与两个附加配置、refdiag再次构建通过。第三次烧录前完整1MiB备份SHA-256为`7dec51e40fcba5b4ab253199b81f9317bdb53ca35b0672e9bff0d50767deb214`，参数切片仍与初始独立备份一致。所有失败观察均保留，不报告为硬件通过。


## 最终固件与首次完整裸板验收

固件源提交`ad41d3204b60b3f928dc972e38b76fec818c9ed0`，禁热Debug、独立refdiag bench_safe，ELF SHA-256 `e0737bd4a60bf06878a6acf56bbea196a85466d247687e344a6eb734d4980d63`。烧录技能再次报告program／verify成功，随后仍在no_save_profile缓存步骤报错；按已报告成功处理，没有因此重烧。保持MCU运行独立回读204260字节应用，与ELF导出的线性二进制逐字节一致；最终实际应用摘要以[固件会话](REF-03-evidence/primary/session.json)为准。256KiB参数区逐字节保留，仍为`413f4da1afa9483a6bf156cb7124f05276927eb843ce5d040c85725257120de4`。

被动观察67.3926秒，13份运行态只读记录：启动诊断READY、BMI088阶段9、原始样本有效递增；13476帧SP全部CRC正确，四元数全有限，模式全0，heater=0；固定GPIO／TIM3输出隔离检查通过，未出现新增IWDG复位或计数回退。四元数范数约0.9983135，只作记录，不作为姿态精度保证。

随后运行有限负载：首30秒满长127字节交互，后45秒混合空操作UI与交互；工具总计80秒采集并包含65秒停更1秒、70秒身份3→2。收到2171份完整线上帧，CRC和固定内容正确；MCU共2174条通知，其中完成2171、身份取消3，错误／过期／隔离均0。满长阶段870帧、混合阶段1300帧，其余为阶段结束后的队列收尾。最小MCU启动间隔34ms，UI最小136ms，目标滚动1000ms最大3683字节；序号与入队编号对应，8次255→0回绕，取消不回退序号。

PC停更窗口65.6～66.0秒无新帧，70.3秒后没有旧身份3帧；这些PC时间仅作粗略交叉观察，预算使用MCU时刻。最后查询accepted=2174、completed=2171、failed=3且细分cancelled=3，used／active／isolated=0、四条UART lease全0。限流等待45833是轮询次数，不代表丢帧数。

结束后额外被动6.86秒，1370份SP帧仍全部CRC正确、有限、模式0，IMU继续递增且无复位／输出开放。诊断phase=3，有限负载已停止，不会自动重新开始。

```sh
python tools/referee_budget_capture.py --port COM3 \
  --elf build/ref03-hardware/verified/Basic_Framework_MC02_refdiag_bench_safe.elf \
  --session build/ref03-hardware/verified/session.json \
  --output build/ref03-hardware/verified/load
```

原始串口、逐帧CSV、MCU读前／读后版本与原始字、完成记录、身份注入及统计保存于[primary证据](REF-03-evidence/primary/load/report.json)。压缩文件可解压后用工具report_capture重新核对。记录读取不要求写者停止，只取读取前已发布且读后仍未绕环覆盖的槽位；记录不完整就拒绝。

采集中曾因进行中文件大小尚未刷新而误判返回链路，完整关闭后的原始字节和通知对应证明接收正常；用户已检查接线并要求重试，因此保持同一SHA、仅复位重启有限诊断，再保存独立一轮结果。没有因这次复测重新烧录或改变参数。


## 接线复查后的重复验收

用户明确要求重试后，仅执行reset run重启同一ad41d32诊断固件，没有再烧录。重新等待启动完成，再连续被动66.9163秒：13份只读记录、13382帧SP均CRC正确、有限、模式0，加热与输出隔离正常，无新增复位迹象。

第二轮有限负载收到2174份完整正确帧，MCU共2177条通知（2174完成、3身份取消），发送错误／过期／隔离均0；满长阶段870、混合阶段1303，其余1帧为收尾。最小启动34ms、UI最小136ms，滚动最大3683字节，序号8次回绕且与请求编号对应。身份停更和变更的粗略PC时间交叉观察再次通过。末尾队列、活动、隔离及四条UART lease均0，phase=3。

结束后6.86秒的1371帧SP继续CRC正确、有限、模式0，IMU序号相对本轮基线继续增长，无新增IWDG复位／输出开放。两轮独立原始数据、MCU原始字与通知、会话及统计见[证据入口](REF-03-evidence/README.md)；记录按SHA-256清单检查，压缩原始数据重新解码及报告核对通过。复测仍没有真实客户端、RF链路或精确物理波形保证。

裸板最终保留禁热refdiag固件，有限负载已结束；没有自动恢复流量，没有连接或初始化执行器。调试观测服务结束后释放DAPLink连接，MCU继续运行。REF-03全部本轮验收完成，MC02-0022及MC02-0095只记录已提供的共享基础，整体仍由原TODO跟踪。
