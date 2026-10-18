# 业务文件注释检查清单（MC02-0107）

MC02-0076将原user_lib拆为标量／角度、三维向量和采样平均入口。表内原文件名和行号保留检查时的含义，链接转到当前对应文件；当前用法见[算法入口](../../../modules/algorithm/algorithm.md)。

MC02-0079迁移后，六个旧CRC文件的链接指向当前统一入口；原文件名、行号、讨论编号和检查结论保留为当时依据，当前用法见[教学20](../../学习笔记/20-SP固定帧与CRC校验.md)。

本清单供你选择需要补注释的文件。检查对象为 `8111ccf4ce4e67a12daf5f6ec8a1967a6041fb7f` 中已跟踪的业务 C/C++ 源码和头文件，共 **178 份**。本轮只新增报告和配套教学说明，业务源码保持原样；后续补注释的文件由你选定。任务记录见 [MC02-0107](../../../.Doc/TODO.md)，规范见 [student-code-comments](../../../.agents/skills/student-code-comments/SKILL.md)。

## 检查结果与读法

**164 份有明确注释问题：96 份建议重点查看，68 份为一般补充；另有 9 份空文件需要确认用途，5 份暂未发现明确问题。** “重点”表示容易误解用法、处理顺序或失败结果；“一般”主要涉及用途、语言和局部说明。它们表示阅读时的影响程度，不表示功能故障等级，也不表示已批准修改。

| 目录 | 文件总数 | 重点 | 一般 | 空文件待确认 | 暂未发现问题 |
|---|---:|---:|---:|---:|---:|
| application | 23 | 14 | 6 | 3 | 0 |
| bsp | 40 | 25 | 15 | 0 | 0 |
| modules | 115 | 57 | 47 | 6 | 5 |
| **合计** | **178** | **96** | **68** | **9** | **5** |

逐文件查看了用途说明、数据定义、接口注释和关键处理代码，涉及行为的条目结合实现及必要调用处核实。检查是否说明清楚，不按注释行数打分，也不因缺少 `@file` 标签就单独判为问题。简单赋值无需逐行解释；字库只检查整体用途、字符对应和数据排列；空文件没有函数可查。

范围包含未启用模块。Core、USB_DEVICE、第三方库、测试、工具脚本、构建配置和文档正文不计入这 178 份；为核实说明而读取的调用处或测试也不另外计数。原文中少量难懂说法只作为问题定位引用，改写建议沿用通俗中文。

### 文件来源怎样判断

比较起点为 `14ea0f4`（2026-01-17）。当前文件中，66 份在此之后新增或移入，70 份原有文件后来改过，42 份内容未变。70 份中有两份 ADC 文件原本为空，单列为“补实现”。

这条时间线便于优先检查后续工作，**不能直接证明文件由 AI 编写**。移植文件可能来自人工实现，同一文件也可能同时有原作者代码和后续辅助修改；原有 daemon 文件甚至保留了提及 Copilot 的注释。因此保留“后续新增／移入、混合修改、原有沿用”的可核对分类，无法确认具体编写来源的均视为待确认。原作者和许可信息应保留。

每行“新增／移入／补实现”提交说明文件何时进入本轮比较范围；“位置”提交来自所链接行的 `git blame`，未另写位置时与新增／移入提交相同；“最近”指文件在检查前的最后一次提交。可用 `git show <提交> -- <文件>` 查看上下文。位置提交只证明该行的来历，不证明整条问题由谁造成。

## 后续新增、移入及原空文件补实现：64 份问题文件

优先核对后续开发内容。移入来源有直接说明的例子包括 WS2812 的板级例程说明、DR 的 Restar2027Sentinel 说明以及配置文件中的远程版本说明；其余不凭名称猜测作者。

| 序号 | 文件与位置 | 来源依据 | 建议程度 | 具体问题 | 建议补充或改写范围 |
|---|---|---|---|---|---|
| 004 | [application/chassis/chassis_motion.c:38](../../../application/chassis/chassis_motion.c#L38) | 后续新增 `6896a52` | 重点 | 无文件用途和内部函数说明；失败先清零、四轮同比减速及最多四次处理舍入误差的循环均无解释。 | 说明输入检查、四轮速度共同缩小及循环退出原因。 |
| 005 | [application/chassis/chassis_motion.h:49](../../../application/chassis/chassis_motion.h#L49) | 后续新增 `6896a52` | 重点 | 公开计算入口只有英文一句话，未说明返回值、参数条件和空指针；各组参数含义不完整。 | 用中文 Doxygen 说明几何参数、速度单位、模式和失败清零。 |
| 010 | [application/config/config.h:134](../../../application/config/config.h#L134) | 后续移入 `19573c7` | 一般 | 配置入口无 Doxygen，未说明返回对象及调用顺序；配置字段和增量系数的单位需要结合实现才能理解。 | 说明配置获取前提、返回地址有效期及必要单位。 |
| 011 | [application/config/sentinel_robot/sentinel_config.c:19](../../../application/config/sentinel_robot/sentinel_config.c#L19) | 后续移入 `19573c7` | 一般 | 已有移植来源说明，但轮距、轮半径和控制参数等成组配置缺少现场使用条件及单位提示。 | 在配置分组旁说明单位和仍需按实车确认的内容，接口说明集中在头文件。 |
| 012 | [application/config/sentinel_robot/sentinel_config.h:3](../../../application/config/sentinel_robot/sentinel_config.h#L3) | 后续移入 `19573c7` | 一般 | 仅有包含和声明，未说明哨兵配置入口返回什么及返回数据如何使用。 | 补简短文件说明和 SentinelConfigInit 的中文接口注释。 |
| 015 | [application/init_diagnostic.c:38](../../../application/init_diagnostic.c#L38) | 后续新增 `ba149d5` | 重点 | 无文件或内部函数说明，首次失败保留、失败次数继续累加及关中断复制数据的原因均未说明。 | 解释为何只记第一次失败，以及读取多字段时避免被中断改写。 |
| 016 | [application/init_diagnostic.h:63](../../../application/init_diagnostic.h#L63) | 后续新增 `ba149d5` | 重点 | 诊断数据只有英文导出说明，多个公开函数无接口注释，编号和计数含义未解释。 | 补各入口、首次失败保留规则、空输出指针和计数用途。 |
| 019 | [application/robot_def.c:38](../../../application/robot_def.c#L38) | 后续移入 `19573c7` | 一般 | 三个 Feed 函数直接使用 robot_instance，但文件说明未提示必须先调用 RobotConfigInit。 | 补先初始化再取配置的前提；完整接口用法放 config.h。 |
| 024 | [bsp/adc/bsp_adc.c:15](../../../bsp/adc/bsp_adc.c#L15) | 原为空；补实现 `904fc2f`；位置 `904fc2f` | 重点 | PA5 注释仍为“预留/其他采样”，实现已采按键数据；DMA 已完成半段、缓存处理、平均循环和样本失效选择缺少说明。 | 校正采样用途，解释只处理已完成部分及数据能否使用的检查。 |
| 025 | [bsp/adc/bsp_adc.h:51](../../../bsp/adc/bsp_adc.h#L51) | 原为空；补实现 `904fc2f`；位置 `904fc2f` | 重点 | 公开入口无完整中文说明；20ms 时限、各返回状态、空参数及原始采样有效与电压换算可用的区别不清楚。 | 补采样单位、状态含义、输出内容及错误时的用法。 |
| 031 | [bsp/crash/bsp_crash.c:380](../../../bsp/crash/bsp_crash.c#L380) | 后续新增 `e7ec93e`；位置 `8682aaf` | 一般 | 存在“四重判据”“栈快照”等已约定替换的表达及部分英文说明；构建识别和记录检查的历史叙述偏长。 | 改成逐项检查记录、读取栈上数据等具体说法，保留异常中先关闭输出的原因。 |
| 032 | [bsp/crash/bsp_crash.h:107](../../../bsp/crash/bsp_crash.h#L107) | 后续新增 `e7ec93e` | 一般 | CrashLogGetLast 未说明空指针、失败时输出是否改写；部分入口参数只在大段说明中出现，含“栈快照”等用词。 | 整理中文 Doxygen，补失败输出和异常入口条件，缩短历史说明。 |
| 033 | [bsp/crash/bsp_crash_frame.c:65](../../../bsp/crash/bsp_crash_frame.c#L65) | 后续新增 `1c4e4dc` | 重点 | 文件仅有英文 SRAM 范围提示；不同异常帧、地址检查和失败先清输出的选择缺少中文说明。 | 解释为何先核对地址和压栈错误，再决定读取普通帧或含浮点寄存器的帧。 |
| 034 | [bsp/crash/bsp_crash_frame.h:12](../../../bsp/crash/bsp_crash_frame.h#L12) | 后续新增 `1c4e4dc` | 重点 | 只有英文用途说明，缺少各输入、成功/失败及输出清零规则；两个输出字段含义未解释。 | 用中文说明地址参数、字数单位、返回值和失败输出。 |
| 035 | [bsp/crash/bsp_crash_guard.c:9](../../../bsp/crash/bsp_crash_guard.c#L9) | 后续新增 `1c4e4dc` | 重点 | 未解释再次进入崩溃记录时为何立即复位，以及请求复位后的无限等待。 | 补文件职责和重复进入、空指针时的处理原因。 |
| 036 | [bsp/crash/bsp_crash_guard.h:7](../../../bsp/crash/bsp_crash_guard.h#L7) | 后续新增 `1c4e4dc` | 重点 | 接口说明为英文，未明确空指针或已经进入时不会正常返回。 | 用中文 Doxygen 说明 active 的初值、修改和复位条件。 |
| 047 | [bsp/param/bsp_param.c:44](../../../bsp/param/bsp_param.c#L44) | 后续新增 `479828a`；位置 `80cf6a3` | 重点 | 无文件职责概述；长时间写入前后的允许/拒绝、失败收尾以及逐块写入循环的说明不足。 | 说明参数 RAM 缓存与 Flash 的关系，补写入前检查和收尾原因。 |
| 048 | [bsp/param/bsp_param.h:67](../../../bsp/param/bsp_param.h#L67) | 后续新增 `479828a`；位置 `80cf6a3` | 重点 | 新增回调说明仅英文，ParamGet/Set 和便捷入口缺参数、字节长度及失败时输出说明。 | 补中文接口说明，明确初始化、暂存/保存区别和写入前必须设置的回调。 |
| 051 | [bsp/safety/bsp_safety.c:18](../../../bsp/safety/bsp_safety.c#L18) | 后续新增 `1c4e4dc` | 重点 | 关键关断说明和函数体注释均为英文，未用中文解释先将输出拉低再切换引脚用途的顺序。 | 补中文文件说明及关键顺序，说明启动调用会恢复原中断状态。 |
| 052 | [bsp/safety/bsp_safety.h:9](../../../bsp/safety/bsp_safety.h#L9) | 后续新增 `1c4e4dc` | 一般 | 启动关断入口仅英文一句话，未用中文说清与保持中断关闭入口的差别。 | 为两个入口集中写简短 Doxygen 和中断状态处理说明。 |
| 053 | [bsp/safety/bsp_safety_error.c:5](../../../bsp/safety/bsp_safety_error.c#L5) | 后续新增 `1c4e4dc` | 重点 | 错误入口无说明；主机测试返回、固件永久等待的条件及关断原因未解释。 | 说明此处提供 Error_Handler 的用途及两个编译分支的行为。 |
| 060 | [bsp/watchdog/bsp_watchdog.c:21](../../../bsp/watchdog/bsp_watchdog.c#L21) | 后续新增 `bffee9f` | 一般 | 无文件用途；等待寄存器更新的20ms限制和未启动时喂狗不做事等选择没有简短说明。 | 补看门狗启动、改时限和复位来源读取的关系，解释等待退出条件。 |
| 061 | [bsp/watchdog/bsp_watchdog.h:6](../../../bsp/watchdog/bsp_watchdog.h#L6) | 后续新增 `bffee9f` | 重点 | 全部公开函数只有声明，未说明超时单位、必须先启动、返回值以及读取复位来源的调用顺序。 | 补中文 Doxygen 和初始化、喂狗、修改时限的使用条件。 |
| 062 | [bsp/ws2812/ws2812.c:79](../../../bsp/ws2812/ws2812.c#L79) | 后续移入 `ef9cab8` | 一般 | 编码循环未说明 GRB 顺序及高位先发，等待 SPI 空闲的循环缺少用途说明。 | 补简短文件说明，并解释编码、等待和发送复位码三个步骤。 |
| 063 | [bsp/ws2812/ws2812.h:8](../../../bsp/ws2812/ws2812.h#L8) | 后续移入 `ef9cab8` | 重点 | 文件写每位用4个SPI bit，实际编码将每个颜色位存成1字节再发送；公开入口也缺颜色范围和调用条件。 | 校正编码描述，补0～255颜色值、初始化前提及发送等待说明。 |
| 066 | [modules/BMI088/bmi088_data.h:10](../../../modules/BMI088/bmi088_data.h#L10) | 后续新增 `4a289b0` | 重点 | 样本字段无单位和轴序说明；5ms检查函数无返回值、空指针及时间条件说明。 | 说明 gyro/acc/温度单位、序号含义和等于5ms仍可使用的条件。 |
| 068 | [modules/BMI088/bmi088_temperature.h:8](../../../modules/BMI088/bmi088_temperature.h#L8) | 后续新增 `3b2c781` | 一般 | 温度换算仅英文，两个寄存器参数、摄氏度返回值及负温度转换含义没有中文说明。 | 用简短 Doxygen 说明输入字节和输出单位，解释负数转换。 |
| 074 | [modules/alarm/led.c:68](../../../modules/alarm/led.c#L68) | 后续移入 `ef9cab8` | 一般 | 闪烁次数、计时重置和停用时显示粉色的选择缺少原因说明，无文件职责概述。 | 解释计时按任务调用次数累计，以及不同报警状态的显示结果。 |
| 075 | [modules/alarm/led.h:43](../../../modules/alarm/led.h#L43) | 后续移入 `ef9cab8` | 一般 | 接口只有声明，未说明先初始化/注册、flash_count 含义及实例使用条件。 | 补文件用途、公开入口和闪烁次数的实际含义。 |
| 078 | [modules/algorithm/RLS.c:110](../../../modules/algorithm/RLS.c#L110) | 后续移入 `47133e8` | 一般 | 文件无用途说明，更新过程多为逐行英文矩阵操作，未先说清输入用于修正哪组参数。 | 简述递推估计用途，按主要计算阶段写简短中文，减少逐行复述。 |
| 079 | [modules/algorithm/RLS.h:44](../../../modules/algorithm/RLS.h#L44) | 后续移入 `47133e8` | 重点 | 四个公开函数只有声明；数组长度、初始化顺序及 delta/lambda 用途不清楚。 | 补输入输出、数组大小和参数条件，解释结果保存在实例何处。 |
| 100 | [modules/imu/imu_heater.c:331](../../../modules/imu/imu_heater.c#L331) | 后续新增 `bffee9f`；位置 `1a97492` | 重点 | 安全策略集中在文件头，样本序号检查、重复数据不重新采用、恢复后等待新数据等函数体分支缺少解释。 | 在重要返回和状态变化旁说明关闭/恢复原因，精简抽象表述，保留全部数值条件。 |
| 101 | [modules/imu/imu_heater.h:32](../../../modules/imu/imu_heater.h#L32) | 后续新增 `bffee9f` | 重点 | 大部分公开入口无注释；目标温度范围、禁热默认值、长操作前后调用及强制关闭条件未在接口说明。 | 补参数单位、调用顺序、返回结果和清故障后等待新数据的规则。 |
| 104 | [modules/ist8310/ist8310.c:63](../../../modules/ist8310/ist8310.c#L63) | 后续新增 `9174e4e` | 重点 | 分配 RAM 后注释写“清除flash”；写配置后的读取被说成“读回自身id”，实际读配置寄存器；失败循环注释又称返回错误码。 | 校正对象、读回内容和失败后的真实动作，删去无助理解的调侃。 |
| 105 | [modules/ist8310/ist8310.h:50](../../../modules/ist8310/ist8310.h#L50) | 后续新增 `9174e4e` | 一般 | 初始化注释缺参数、成功/失败和部分失败会持续停留的行为；文件用途和三轴磁场单位未集中说明。 | 补中文 Doxygen，准确说明已实现的失败处理及数据单位。 |
| 106 | [modules/lcd/lcd.c:4](../../../modules/lcd/lcd.c#L4) | 后续新增 `cdf45a3` | 一般 | 文件缺整体职责；大量公开接口的完整说明只在 .c 中，头文件未集中维护；字体绘制的多层循环多直接复述动作。 | 接口用法集中到头文件，源文件简述字体取位、画点和背景选择的原因。 |
| 107 | [modules/lcd/lcd.h:64](../../../modules/lcd/lcd.h#L64) | 后续新增 `cdf45a3` | 重点 | 多处行尾中文在文件中已呈“LCD鍒濆…”等不可读文字；接口缺坐标范围、颜色格式、字体大小和模式说明。 | 恢复可读中文，集中补绘图接口 Doxygen 和参数单位。 |
| 108 | [modules/lcd/lcd_buf.c:136](../../../modules/lcd/lcd_buf.c#L136) | 后续新增 `cdf45a3` | 重点 | 无文件用途；按行拼图再发送、字符取位和中文结束检查的循环缺少说明。 | 说明行缓冲减少发送次数，并解释字体排列、边界返回和小数转换。 |
| 109 | [modules/lcd/lcd_buf.h:7](../../../modules/lcd/lcd_buf.h#L7) | 后续新增 `cdf45a3`；位置 `ca59d8a` | 重点 | 绘图入口均无注释；坐标包含哪一端、字号、颜色格式、中文结束标记和越界不绘制未说明。 | 补中文 Doxygen、像素单位及输入不合要求时的结果。 |
| 110 | [modules/lcd/lcd_font_cn.c:3](../../../modules/lcd/lcd_font_cn.c#L3) | 后续新增 `cdf45a3` | 一般 | 各字已有标签，但文件未说明16×16字模的排列、与枚举顺序的关系及用途。 | 只补字库整体用途和读取方式，不逐字节加注释。 |
| 111 | [modules/lcd/lcd_font_cn.h:6](../../../modules/lcd/lcd_font_cn.h#L6) | 后续新增 `cdf45a3` | 一般 | 字形枚举缺对应汉字提示和文件用途，读者需另查 .c 才能对应拼音编号；字模表排列未说明。 | 简述枚举与字模、标签序列的关系，补必要的字符提示。 |
| 112 | [modules/lcd/lcd_key_input.c:28](../../../modules/lcd/lcd_key_input.c#L28) | 后续新增 `6ef4612` | 重点 | 文件无任何注释；样本无效时取消状态、连续三次确认按下/松开、按住时忽略方向变化的选择未解释。 | 在各阶段加简短说明，解释为何先清事件及连续确认才报告。 |
| 113 | [modules/lcd/lcd_key_input.h:39](../../../modules/lcd/lcd_key_input.h#L39) | 后续新增 `6ef4612` | 重点 | 更新接口与状态结构无说明；首次需清零、时间单位、事件输出和无效采样取消按键的规则未交代。 | 补中文 Doxygen、状态初始化及按下/松开事件含义。 |
| 114 | [modules/lcd/lcd_task.c:104](../../../modules/lcd/lcd_task.c#L104) | 后续新增 `cdf45a3`；位置 `ff71038` | 一般 | 含“CPU占用快照”等词；任务统计的多层查找、屏幕在位检查中的连续确认与跳过分支说明不足。 | 补任务文件职责，解释循环处理对象及重连确认，把诊断用词改为具体内容。 |
| 115 | [modules/lcd/lcd_task.h:4](../../../modules/lcd/lcd_task.h#L4) | 后续新增 `cdf45a3` | 一般 | 唯一任务入口没有注释，未说明职责、参数未使用及任务创建方式。 | 补简短文件说明和 StartLCDTASK 的中文 Doxygen。 |
| 116 | [modules/lcd/lcd_ui.c:127](../../../modules/lcd/lcd_ui.c#L127) | 后续新增 `cdf45a3`；位置 `4049f99` | 一般 | 大量内部绘图函数无简短用途，部分只靠名字理解；页面选择、相同内容不重画等流程缺文件级串联。 | 说明静态页面与数值刷新的关系，补复杂内部函数及关键选择，保留已有重绘原因。 |
| 117 | [modules/lcd/lcd_ui.h:8](../../../modules/lcd/lcd_ui.h#L8) | 后续新增 `cdf45a3` | 一般 | 多数公开入口无说明；page 范围、frozen 行为、初始化与恢复先后关系不清楚。 | 补中文 Doxygen 和任务调用条件，保留缓存失效说明并核对调用处。 |
| 118 | [modules/lcd/lcdfont.h:4](../../../modules/lcd/lcdfont.h#L4) | 后续新增 `cdf45a3` | 一般 | 字模有逐字符标记，但缺文件用途、字符起点以及按行/列和位顺序读取的总说明。 | 只补各尺寸字库排列与对应绘制入口，保留原有数据。 |
| 125 | [modules/master_machine/vision_protocol.c:40](../../../modules/master_machine/vision_protocol.c#L40) | 后续新增 `d0810e4` | 重点 | 无文件用途；逐位置找 SP、CRC 错误继续查找和找到第一帧即返回的循环缺少中文说明。 | 解释寻找范围、为何跳过坏帧、返回哪一帧及错误计数。 |
| 126 | [modules/master_machine/vision_protocol.h:47](../../../modules/master_machine/vision_protocol.h#L47) | 后续新增 `d0810e4` | 重点 | CRC 入口无接口说明，找帧仅英文；协议字段缺模式和角度/速度单位，失败时帧输出保持不变未说明。 | 补中文 Doxygen、长度含不含 CRC、帧字段单位与空指针/失败行为。 |
| 129 | [modules/message_center/message_freshness.h:15](../../../modules/message_center/message_freshness.h#L15) | 后续新增 `d2a503b` | 重点 | 时间检查函数和字段均无注释，未说明50ms默认值、等于时限允许以及空指针返回0。 | 补用途、毫秒时间来源及函数使用条件。 |
| 135 | [modules/motor/DRmotor/drmotor.h:1](../../../modules/motor/DRmotor/drmotor.h#L1) | 后续移入 `dc6b5c6` | 一般 | 整个头文件都被注释，但没有像配套 .c 一样说明当前未实现和保留来源，容易被当成可用接口。 | 只补一句占位用途及未启用说明，不给注释掉的声明逐个扩写。 |
| 140 | [modules/motor/XMmotor/xmmotor.c:86](../../../modules/motor/XMmotor/xmmotor.c#L86) | 后续移入 `dc6b5c6` | 一般 | 文件未整体说明默认只保留软件结构、硬件未启用；部分复杂协议函数没有用途说明，任务模式选择解释不全。 | 说明 XM_MOTOR_ENABLE 的实际影响，并补模式与协议处理的关键步骤。 |
| 141 | [modules/motor/XMmotor/xmmotor.h:62](../../../modules/motor/XMmotor/xmmotor.h#L62) | 后续移入 `dc6b5c6` | 重点 | 接口无注释，未告知默认未注册 CAN/未创建任务；参数、单位及置零等行为不明确。 | 补当前可用范围、初始化顺序和各入口说明，不把待实现硬件功能写成保证。 |
| 152 | [modules/power/power_manager.c:26](../../../modules/power/power_manager.c#L26) | 后续新增 `c123dbf` | 重点 | 反馈与更新函数实际为空，文件未明确目前只保留结构、没有功率估计或分配；读者容易按函数名误解。 | 补当前实现程度和空函数用途，保留被注释代码的参考身份。 |
| 153 | [modules/power/power_manager.h:24](../../../modules/power/power_manager.h#L24) | 后续新增 `c123dbf` | 重点 | 公开入口没有说明，未告知反馈/更新尚未实现；k1/k2/k3 和功率输入的含义未解释。 | 补中文接口说明，明确哪些入口仅占位，不承诺尚未实现的控制效果。 |
| 163 | [modules/remote/remote_bench.c:15](../../../modules/remote/remote_bench.c#L15) | 后续新增 `0951f81` | 重点 | 裸板诊断代码无说明，allow_enable=0 与每轮请求失能的目的未解释。 | 说明只观察遥控与手势、持续禁止使能，以及宏开关的适用条件。 |
| 164 | [modules/remote/remote_bench.h:4](../../../modules/remote/remote_bench.h#L4) | 后续新增 `0951f81`；位置 `ba149d5` | 重点 | 两个入口没有职责、参数、返回值或仅在裸板诊断配置中可用的说明。 | 补中文 Doxygen 和配置要求。 |
| 167 | [modules/remote/remote_safety.c:10](../../../modules/remote/remote_safety.c#L10) | 后续新增 `0951f81` | 重点 | 无任何用途注释；保持一秒的检查、离线重置和三个开关组合的动作未解释。 | 说明保持计时、触发后重新开始计时及禁止使能时的处理。 |
| 168 | [modules/remote/remote_safety.h:7](../../../modules/remote/remote_safety.h#L7) | 后续新增 `0951f81` | 重点 | 仅英文一句，缺 state 初始化、时间单位、输入可为空和各手势含义。 | 补中文 Doxygen、指针要求及一秒触发条件。 |
| 169 | [modules/safety/robot_safety.c:214](../../../modules/safety/robot_safety.c#L214) | 后续新增 `bffee9f` | 重点 | 无文件职责，状态选择优先顺序和不同故障是否保留使能请求的说明不足，关键说明仍为英文。 | 解释初始化失败、急停、任务超时、标定和普通故障的处理顺序。 |
| 170 | [modules/safety/robot_safety.h:29](../../../modules/safety/robot_safety.h#L29) | 后续新增 `bffee9f` | 重点 | 多数公开接口只有声明；故障行为仅英文段落，清急停的返回值和恢复条件未说明。 | 补中文 Doxygen，说明上报后需 Update、哪些请求保留及清除失败原因。 |
| 175 | [modules/task_monitor/task_monitor.c:142](../../../modules/task_monitor/task_monitor.c#L142) | 后续新增 `bffee9f` | 重点 | 无文件职责，大量任务与长操作检查循环没有解释；为何只在全部任务正常时喂狗、写 Flash 期间如何计时不易理解。 | 补关键循环对象、长操作到期和故障保留原因，替换英文说明。 |
| 176 | [modules/task_monitor/task_monitor.h:30](../../../modules/task_monitor/task_monitor.h#L30) | 后续新增 `bffee9f` | 重点 | 多数入口无说明，长操作只有英文；任务各时限、初始化、延时会更新谁的计数及输出失败行为未交代。 | 补中文 Doxygen、毫秒单位、调用频率、长操作限时与返回值。 |

## 原有代码经后续修改：67 份问题文件

这些文件应结合新旧内容阅读。优先校正说明与实现不一致、以及后来新增但未解释的接口；原有准确说明继续保留。

| 序号 | 文件与位置 | 来源依据 | 建议程度 | 具体问题 | 建议补充或改写范围 |
|---|---|---|---|---|---|
| 002 | [application/chassis/chassis.c:167](../../../application/chassis/chassis.c#L167) | 混合修改；位置 `6c2f74b`；最近 `08cc241` | 一般 | “后续增加没收到消息的处理”已落后于下面的消息超时清零；新加的模式选择、计算失败停轮缺少对应说明。 | 更新过时说明，在命令过期及计算失败处解释停轮和清除旧速度。 |
| 003 | [application/chassis/chassis.h:18](../../../application/chassis/chassis.h#L18) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | ChassisInit 已返回 uint8_t，注释未解释成功、失败；电机信息输出参数及空指针处理也未说明。 | 补文件用途、初始化返回值和 Chassis_GetMotorSummary 的输出用法。 |
| 008 | [application/cmd/robot_cmd.c:209](../../../application/cmd/robot_cmd.c#L209) | 混合修改；位置 `6c2f74b`；最近 `08cc241` | 重点 | 右开关向下的注释写“底盘跟随云台”，代码选 CHASSIS_ROTATE；视觉退出后的手动角度接续等重要选择说明不足，部分新说明只有英文。 | 校正档位说明；解释命令来源、反馈失效和视觉退出时的处理。 |
| 009 | [application/cmd/robot_cmd.h:10](../../../application/cmd/robot_cmd.h#L10) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | RobotCMDInit 的成功/失败返回值未说明；文件用途、依赖初始化顺序不完整。 | 补初始化返回值和 RobotCMDTask 调用条件。 |
| 013 | [application/gimbal/gimbal.c:106](../../../application/gimbal/gimbal.c#L106) | 混合修改；位置 `6c2f74b`；最近 `08cc241` | 重点 | “后续增加未收到数据的处理”与现有消息超时清零不符；无文件用途，新的失效处理没有解释。 | 更新过时说明，补命令失效及反馈模式切换的简短说明。 |
| 014 | [application/gimbal/gimbal.h:16](../../../application/gimbal/gimbal.h#L16) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | GimbalInit 未解释 uint8_t 返回值；摘要输出参数及空指针行为未说明。 | 补文件职责、初始化成败和输出参数用法。 |
| 017 | [application/robot.c:18](../../../application/robot.c#L18) | 混合修改；位置 `80cf6a3`；最近 `ba149d5` | 重点 | 无文件职责说明；参数写入前后的加热停止、任务等待时间配合及失败退出缺少函数内说明。 | 说明启动顺序和长时间操作前后检查，解释失败后进入何种运行状态。 |
| 018 | [application/robot.h:11](../../../application/robot.h#L11) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | RobotInit 已有成功/失败返回值，接口注释未解释；RobotTask 的调用条件不完整。 | 补返回值和初始化失败后任务行为。 |
| 020 | [application/robot_def.h:38](../../../application/robot_def.h#L38) | 混合修改；位置 `59f6b4d`；最近 `d2a503b` | 一般 | 视觉范围设置说明仍为英文；控制消息中的角速度、角加速度等字段单位说明不完整。 | 用中文说明范围配置条件，并核对控制消息字段单位。 |
| 021 | [application/robot_task.h:45](../../../application/robot_task.h#L45) | 混合修改；位置 `ba149d5`；最近 `08cc241` | 重点 | OSTaskInit 未说明返回值，任务创建失败和常驻循环的职责说明不统一；部分长注释停留在修改过程。 | 说明任务启动条件、失败记录和循环周期，压缩历史排错叙述。 |
| 022 | [application/shoot/shoot.c:122](../../../application/shoot/shoot.c#L122) | 混合修改；位置 `d2a503b`；最近 `d2a503b` | 重点 | 命令过期后必须等 LOAD_STOP 才能重新单发/三发的分支缺少说明；require_load_stop 的英文行尾注释仍写 last load mode。 | 解释停止等待和重新允许单发的条件，把供弹模式说明改为简明中文。 |
| 023 | [application/shoot/shoot.h:17](../../../application/shoot/shoot.h#L17) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | ShootInit 未解释返回值；电机信息输出参数及空指针处理未说明。 | 补文件用途、初始化成败和信息读取接口。 |
| 026 | [bsp/bsp_init.h:15](../../../bsp/bsp_init.h#L15) | 混合修改；位置 `6c2f74b`；最近 `80cf6a3` | 一般 | 调用者误写为 RobotoInit；文件只写初始化入口，未直观交代本头文件带实现及包含要求。 | 改正调用名称，补一句仅由机器人初始化代码包含的用途说明。 |
| 029 | [bsp/can/bsp_can.c:11](../../../bsp/can/bsp_can.c#L11) | 混合修改；位置 `6c2f74b`；最近 `d2a503b` | 重点 | 开头仍写接收中断直接遍历并调用回调，当前实现已由中断入队、任务派发；有“你不需要完全理解”等不适合学生的说明。 | 校正收发调用关系，解释查表失败、队列满和恢复选择，压缩历史描述。 |
| 030 | [bsp/can/bsp_can.h:170](../../../bsp/can/bsp_can.h#L170) | 混合修改；位置 `6c2f74b`；最近 `d2a503b` | 重点 | CANTransmit 文档仍围绕等待超时，当前发送会在队列满时直接失败；返回值、注册失败及部分输出参数说明不完整。 | 核对非等待发送、回调运行位置与指针使用条件，用中文说明参数和成败。 |
| 037 | [bsp/dwt/bsp_dwt.c:8](../../../bsp/dwt/bsp_dwt.c#L8) | 混合修改；位置 `6c2f74b`；最近 `6bfaaea` | 一般 | 文件 @brief 留空；时间累加说明包含较长排错历史，延时循环没有简短用途说明。 | 补计时文件职责，保留计数绕回及避免读写互相打断的原因，压缩历史。 |
| 038 | [bsp/dwt/bsp_dwt.h:142](../../../bsp/dwt/bsp_dwt.h#L142) | 混合修改；位置 `6c2f74b`；最近 `08cc241` | 重点 | 写“会被三个 timeline 函数调用”，实际三者调用 DWT_CyclesUpdate；频率例子仅旧板，新增计时入口缺少各自使用说明。 | 校正调用关系，补当前板频率、传入计数会被更新及计数绕回前须调用的条件。 |
| 044 | [bsp/iic/bsp_iic.h:100](../../../bsp/iic/bsp_iic.h#L100) | 混合修改；位置 `6c2f74b`；最近 `c0d1cb3` | 重点 | 写“目前只支持8位地址”，实现按 mem8bit_flag 选择8或16位；异步收发所用数据需要保留多久未说明。 | 校正地址宽度说明，补长度单位和异步数据有效期。 |
| 045 | [bsp/log/bsp_log.c:17](../../../bsp/log/bsp_log.c#L17) | 混合修改；位置 `4d68120`；最近 `a8f4364` | 一般 | 文件无用途说明；传入空的限速记录会允许打印、到时才更新计数的选择缺少解释；自测历史说明较长。 | 简述初始化与格式化关系，解释限速选择并压缩自测背景。 |
| 046 | [bsp/log/bsp_log.h:133](../../../bsp/log/bsp_log.h#L133) | 混合修改；位置 `6c2f74b`；最近 `a8f4364` | 重点 | PrintLog 注释写打印字符数，实际返回格式化所需长度，长文本可能未全部输出；Float2Str 仍写浮点必须先转换，与 newlib 路径不符。 | 校正返回值及浮点说明，补缓冲区要求、空限速记录行为，替换“截断”等用词。 |
| 049 | [bsp/pwm/bsp_pwm.c:61](../../../bsp/pwm/bsp_pwm.c#L61) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | 新增注册失败清理、启动前占空比清零、注销时移除数组项的原因未说明；文件缺用途说明。 | 解释为何先保持无输出再启动，以及失败清理和注销循环。 |
| 050 | [bsp/pwm/bsp_pwm.h:50](../../../bsp/pwm/bsp_pwm.h#L50) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | PWMUnregister 无注释，注册失败返回 NULL 未说明；文件 @brief 为空，模板作者仍为 your name。 | 补文件用途、注册/注销结果、周期和频率单位；保留可确认的原作者信息。 |
| 054 | [bsp/spi/bsp_spi.c:50](../../../bsp/spi/bsp_spi.c#L50) | 混合修改；位置 `afe436e`；最近 `ba149d5` | 重点 | 新增总线占用、超时等待、HAL tick 检查多为英文说明；关键失败释放和传输模式分支缺少统一中文解释。 | 解释谁在使用总线、等待何时结束、失败为何归还使用权。 |
| 055 | [bsp/spi/bsp_spi.h:57](../../../bsp/spi/bsp_spi.h#L57) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | SPIUnregister 无说明；新增等待条件只有英文，异步发送数据保存时间、失败返回及调用条件不完整。 | 补中文 Doxygen，分别说明同步完成与异步启动、数据使用时间和注销要求。 |
| 056 | [bsp/usart/bsp_usart.c:332](../../../bsp/usart/bsp_usart.c#L332) | 混合修改；位置 `0951f81`；最近 `d2a503b` | 重点 | RxEvent 的参数说明紧贴 USARTGetCurrentRxData 定义，说明与函数错位；错误回调仍写重新启动，实际只记录待恢复。 | 移动和校正说明，压缩长历史段落，保留入队、派发、恢复的原因。 |
| 057 | [bsp/usart/bsp_usart.h:55](../../../bsp/usart/bsp_usart.h#L55) | 混合修改；位置 `ba149d5`；最近 `d2a503b` | 重点 | 服务初始化未解释返回值；USARTSend 漏 mode 参数；当前帧指针及时间读取仅英文/无说明。 | 补注册失败、数据可使用的时间、发送模式及时间单位。 |
| 058 | [bsp/usb/bsp_usb.c:104](../../../bsp/usb/bsp_usb.c#L104) | 混合修改；位置 `120fc28`；最近 `d2a503b` | 一般 | 派发循环未说明旧数据为何跳过；当前数据读取入口无说明，头部仍保留模板作者。 | 补收发文件职责和过期不派发的原因，接口细节集中在头文件。 |
| 059 | [bsp/usb/bsp_usb.h:28](../../../bsp/usb/bsp_usb.h#L28) | 混合修改；位置 `6c2f74b`；最近 `d2a503b` | 重点 | USBInit 注释声称会重新枚举，实际只设置回调并取得接收地址；两个当前帧读取入口无用法说明。 | 校正初始化行为，说明发送数据需保留多久、接收指针只在回调中可用及返回状态。 |
| 064 | [modules/BMI088/bmi088.c:648](../../../modules/BMI088/bmi088.c#L648) | 混合修改；位置 `041e99d`；最近 `08cc241` | 重点 | 含“判据”等约定替换词和长排错历史；完整样本发布、错误重试及标定超时的关键新增路径说明不均衡。 | 按采样、校验、发布和标定分段解释，说明失败为何保留旧值并报告失败。 |
| 065 | [modules/BMI088/bmi088.h:172](../../../modules/BMI088/bmi088.h#L172) | 混合修改；位置 `041e99d`；最近 `ab8fd5f` | 重点 | 标定返回值仍用“严格判据”等词；按需标定写长按中键，当前按键处理已改为左右长按；注册失败和采样状态说明不全。 | 校正触发方式，补成功/失败、状态含义和调用前提，简化术语。 |
| 067 | [modules/BMI088/bmi088_regNdef.h:182](../../../modules/BMI088/bmi088_regNdef.h#L182) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 一般 | 寄存器分组已有说明，但传感器换算系数未集中说明角速度、加速度和温度的输出单位，部分分组仅英文。 | 补文件用途及系数组单位提示，保持寄存器宏原样。 |
| 072 | [modules/alarm/buzzer.c:91](../../../modules/alarm/buzzer.c#L91) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 重点 | 无文件职责；遍历报警等级、跳过空项以及选择响声的循环缺少解释，初始化失败的清理也未说明。 | 按实际优先级顺序说明循环与分支，说明初始化失败不留下已注册 PWM。 |
| 073 | [modules/alarm/buzzer.h:43](../../../modules/alarm/buzzer.h#L43) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | 公开接口均无用法说明；loudness 允许范围、等级独占、初始化返回值和默认对象未解释。 | 补中文 Doxygen、配置范围和返回 NULL/0 的情况。 |
| 076 | [modules/algorithm/QuaternionEKF.c:7](../../../modules/algorithm/QuaternionEKF.c#L7) | 混合修改；位置 `6c2f74b`；最近 `08cc241` | 一般 | 文件用途及部分关键步骤仅英文，矩阵缩写和卡方检查失败后的选择未用初学者易懂的话串联。 | 保留算法名称和作者，补每阶段目的、何时采用预测结果，接口说明集中到头文件。 |
| 077 | [modules/algorithm/QuaternionEKF.h:72](../../../modules/algorithm/QuaternionEKF.h#L72) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | 初始化和更新函数无接口注释；角速度/加速度/时间步长及输出角度单位、初始化失败未解释。 | 用中文 Doxygen 说明数组大小、参数、单位和返回结果。 |
| 080 | [modules/algorithm/controller.c:146](../../../modules/algorithm/controller.c#L146) | 混合修改；位置 `6c2f74b`；最近 `fa22f0c` | 重点 | PIDCalculate 的源内说明写“返回空”，实际返回 float；内部辅助函数说明密度不一致。 | 校正返回说明并与头文件去重，补积分停止等容易误解的选择原因。 |
| 086 | [modules/algorithm/kalman_filter.c:145](../../../modules/algorithm/kalman_filter.c#L145) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 重点 | 原初始化注释紧贴 Kalman_Filter_Release 定义，参数说明与函数错位；分配失败清理路径缺少说明，文件头示例过长。 | 校正注释位置，说明分配/释放及失败处理，保留必要算法步骤并压缩示例。 |
| 087 | [modules/algorithm/kalman_filter.h:115](../../../modules/algorithm/kalman_filter.h#L115) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | 初始化、释放和更新等公开函数只有声明；维数、初始化失败、返回数据保存位置和有效期未说明。 | 补中文 Doxygen，解释调用顺序、内存释放及各分步入口。 |
| 088 | [modules/algorithm/user_lib.c:175](../../../modules/algorithm/vector3.c) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 一般 | 含“归一化”等词，部分步骤只有英文；AverageFilter 写删除最后一个元素，实际向前移动后替换末项。 | 改为具体向量处理说明，校正均值滤波步骤，补文件用途。 |
| 089 | [modules/algorithm/user_lib.h:107](../../../modules/algorithm/vector3.h) | 混合修改；位置 `6c2f74b`；最近 `47133e8` | 重点 | 向量、矩阵和均值滤波函数无接口说明；是否改写原数组、数组长度、返回地址及有效输入条件不清楚。 | 补中文 Doxygen 和会修改输入的说明，保留角度与长度单位。 |
| 092 | [modules/can_comm/can_comm.c:33](../../../modules/can_comm/can_comm.c#L33) | 混合修改；位置 `d2a503b`；最近 `d2a503b` | 重点 | 新增单帧过期、整包拼接超时、复制前清零输出等选择缺少说明；文件无整体收发流程。 | 解释何时放弃未收齐的数据、CRC通过后才发布及读取失败为何清零。 |
| 093 | [modules/can_comm/can_comm.h:84](../../../modules/can_comm/can_comm.h#L84) | 混合修改；位置 `d2a503b`；最近 `d2a503b` | 重点 | 新增 CANCommGetSnapshot 无任何接口说明；旧指针读取未说明何时会被覆盖，在线返回值仍空泛。 | 补复制读取的参数、时间单位、失败清零和指针读取注意事项。 |
| 094 | [modules/daemon/daemon.c:80](../../../modules/daemon/daemon.c#L80) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 一般 | 末尾保留“下面是copilot的回答”和简单电机例子，长度超出实际说明需要；无文件用途概述。 | 把 owner_id 用途压成一句，补注销循环和只报告一次掉线的说明。 |
| 095 | [modules/daemon/daemon.h:40](../../../modules/daemon/daemon.h#L40) | 混合修改；位置 `ba149d5`；最近 `ba149d5` | 重点 | 注销无说明；owner_id 写成 daemon 实例地址，实际由拥有模块提供；计数单位和0时默认100未解释。 | 补计数周期、注册/注销结果和所属模块地址，说明等待计数不代表已收到有效数据。 |
| 102 | [modules/imu/ins_task.c:225](../../../modules/imu/ins_task.c#L225) | 混合修改；位置 `041e99d`；最近 `ba149d5` | 重点 | 按需标定仍写长按中键；文件 @brief 留空；初始化失败和采样失效跳过解算的新增路径缺少说明。 | 校正按键说明，补采样→姿态→上行数据的关系和失败后的动作。 |
| 103 | [modules/imu/ins_task.h:10](../../../modules/imu/ins_task.h#L10) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 重点 | 文件头要求初始化不要放实时系统，当前已由 INS 专属任务初始化；多处参数为空，角度/时间单位和数据地址使用条件不全。 | 更新初始化时机，补中文返回值、空返回、姿态单位和数组大小。 |
| 119 | [modules/master_machine/master_process.c:169](../../../modules/master_machine/master_process.c#L169) | 混合修改；位置 `d2a503b`；最近 `d2a503b` | 重点 | 已有协议职责，但链路超时清除旧帧、同时读取帧与状态、初始化失败清理等内部函数缺目的说明。 | 解释超时撤销和同一次读取的原因，补内部函数一句话说明。 |
| 120 | [modules/master_machine/master_process.h:97](../../../modules/master_machine/master_process.h#L97) | 混合修改；位置 `d0810e4`；最近 `d2a503b` | 重点 | 公开接口用“原子快照”，读者难理解；VisionUpdateTx 参数单位、输出指针可否为空、初始化返回值未说明。 | 用中文说明一次复制帧与状态、参数单位、发送/更新关系和失联后的结果。 |
| 127 | [modules/message_center/message_center.c:153](../../../modules/message_center/message_center.c#L153) | 混合修改；位置 `d2a503b`；最近 `d2a503b` | 重点 | 新增保留最新消息的读取、时间检查和失败先清零缺少说明；发布到所有订阅者的循环未交代覆盖旧消息规则。 | 说明普通读取与最新值读取的区别、为何不能沿用旧数据及覆盖行为。 |
| 128 | [modules/message_center/message_center.h:88](../../../modules/message_center/message_center.h#L88) | 混合修改；位置 `6c2f74b`；最近 `d2a503b` | 重点 | 发布返回值写订阅者数，实际成功固定返回1；新增读取/带时间发布缺中文用法，PubRegister 漏 data_len。 | 校正返回值，补字节长度、毫秒时间、注册失败和读取失败清零。 |
| 130 | [modules/motor/DJImotor/dji_motor.c:14](../../../modules/motor/DJImotor/dji_motor.c#L14) | 混合修改；位置 `80c272f`；最近 `ba149d5` | 重点 | 发送性能注释仍写含自旋等待，当前 CANTransmit 不等待；控制循环中异常数字清零、停机后清发送项的原因说明不足。 | 校正发送行为，补电机循环和停止/异常时输出处理的解释。 |
| 131 | [modules/motor/DJImotor/dji_motor.h:94](../../../modules/motor/DJImotor/dji_motor.h#L94) | 混合修改；位置 `6c2f74b`；最近 `d6d9050` | 重点 | 注册失败仅写返回类型，参考值随控制环变化的单位不清楚；IsOnline 实际更新标志但接口只写判断，摘要空参数未说明。 | 补注册成败、角度/速度/电流参考单位、在线标志及输出参数。 |
| 136 | [modules/motor/HTmotor/HT04.c:82](../../../modules/motor/HTmotor/HT04.c#L82) | 混合修改；位置 `6c2f74b`；最近 `4d68120` | 一般 | 有情绪化评语，未解释协议中的实际处理原因；任务控制各环选择和持续循环用途说明不完整。 | 替换评语，说明模式切换、反馈来源、控制循环及停止输出。 |
| 137 | [modules/motor/HTmotor/HT04.h:74](../../../modules/motor/HTmotor/HT04.h#L74) | 混合修改；位置 `6c2f74b`；最近 `dc6b5c6` | 重点 | 初始化 @brief 和多个参数为空，SetRef 的参数名写 current，参考值单位和有效实例前提不清楚。 | 补中文文件职责、正确参数名及控制环对应单位。 |
| 138 | [modules/motor/LKmotor/LK9025.c:142](../../../modules/motor/LKmotor/LK9025.c#L142) | 混合修改；位置 `6c2f74b`；最近 `4d68120` | 一般 | “这里随便写的”等措辞未说明发送位置计算；多圈角度跨圈选择和各反馈来源分支解释不足。 | 改为具体 CAN 数据位置、角度累计和控制环选择说明。 |
| 142 | [modules/motor/motor_def.h:94](../../../modules/motor/motor_def.h#L94) | 混合修改；位置 `6c2f74b`；最近 `dc6b5c6` | 一般 | “后续增加前馈数据指针”已与紧随的前馈字段不符；反馈及前馈指针需保持可读、参考值单位随控制环变化未说明。 | 更新过时说明，解释指针由调用方保持有效和控制环的关系。 |
| 143 | [modules/motor/motor_task.c:39](../../../modules/motor/motor_task.c#L39) | 混合修改；位置 `6c2f74b`；最近 `4f3f68b` | 重点 | 提示取消注释才启用对应电机，紧接着 LKMotorControl 已在执行；裸板分支为何跳过控制未说明。 | 补通信派发→电机计算的文件职责，校正启用说明及裸板选择原因。 |
| 146 | [modules/motor/servo_motor/servo_motor.h:44](../../../modules/motor/servo_motor/servo_motor.h#L44) | 混合修改；位置 `6c2f74b`；最近 `6b49953` | 重点 | 无公开接口说明；angle 对总线与 PWM 的含义不同，后者直接传给占空比，不能统一按角度理解。 | 补各类型参数含义、初始化顺序、返回对象和现有实现限制。 |
| 158 | [modules/referee/referee_protocol.h:125](../../../modules/referee/referee_protocol.h#L125) | 混合修改；位置 `471a28b`；最近 `e93aa50` | 一般 | 分组已较清楚，但文件头未交代协议用途与适用版本依据，比赛状态、功率、位置等字段多未写单位和编码含义。 | 补整体说明和必要字段含义，保留数据结构与长度常量，不把本轮检查当作协议升级。 |
| 159 | [modules/referee/referee_task.c:4](../../../modules/referee/referee_task.c#L4) | 混合修改；位置 `6c2f74b`；最近 `cdf45a3` | 一般 | 文件头用途留空；DeterminRobotID 无参数但注释仍列参数，UI刷新重要选择多只有英文分组。 | 说明初始化、等待机器人编号、测试模式和变化后刷新的关系，校正函数说明。 |
| 160 | [modules/referee/referee_task.h:11](../../../modules/referee/referee_task.h#L11) | 混合修改；位置 `6c2f74b`；最近 `cdf45a3` | 重点 | 初始化缺参数、返回结果和 UI_data 长期使用条件；MyUIInit 会等待机器人编号但接口未说明。 | 补初始化顺序、可等待的任务调用位置和输出地址使用规则。 |
| 161 | [modules/referee/rm_referee.c:31](../../../modules/referee/rm_referee.c#L31) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 一般 | 解析函数返回 void，注释却写是否处理正误；文件用途留空，新增初始化失败清理缺说明。 | 校正解析函数说明，补接收→校验→分发关系及失败退出原因。 |
| 162 | [modules/referee/rm_referee.h:102](../../../modules/referee/rm_referee.h#L102) | 混合修改；位置 `6c2f74b`；最近 `cdf45a3` | 重点 | 初始化失败返回 NULL 未说明；发送长度单位、异步数据保存时间和两个状态入口的返回含义不全。 | 补中文 Doxygen、注册前提、发送缓冲条件和在线/最近接收时间含义。 |
| 165 | [modules/remote/remote_control.c:55](../../../modules/remote/remote_control.c#L55) | 混合修改；位置 `5d0e696`；最近 `ba149d5` | 重点 | 新增键鼠允许使用检查和候选数据解码只写英文；150ms到期、递增序号、两次全松开恢复及按键计数循环缺少中文解释。 | 补文件收包流程，在接收拒绝、重新允许键鼠和按键次数更新处说明原因。 |
| 166 | [modules/remote/remote_control.h:148](../../../modules/remote/remote_control.h#L148) | 混合修改；位置 `5d0e696`；最近 `5d0e696` | 重点 | 当前有效输入读取仅英文，初始化返回值、输入地址有效期和键鼠失效清零未说明；部分旧按键说明仍称待测试。 | 补中文 Doxygen、150ms条件及摇杆与键鼠各自可用标志。 |
| 173 | [modules/super_cap/super_cap.c:2](../../../modules/super_cap/super_cap.c#L2) | 混合修改；位置 `6c2f74b`；最近 `ba149d5` | 一般 | 文件描述为空，反馈解码和初始化失败清理无用途说明；接收值仅组合整数，未交代未做单位换算。 | 补收发职责、反馈字段如何保存及失败退出的原因。 |
| 174 | [modules/super_cap/super_cap.h:41](../../../modules/super_cap/super_cap.h#L41) | 混合修改；位置 `6c2f74b`；最近 `c123dbf` | 重点 | 初始化未说明重复注册/失败返回 NULL；发送数据固定8字节未说明，反馈整数的比例与单位未解释。 | 补中文 Doxygen 和原始整数含义；未知设备比例明确待确认。 |

## 基线后未改的原有代码：33 份问题文件

原作者注释中也有需要补充的位置。是否统一调整这些旧文件，同样由你决定。

| 序号 | 文件与位置 | 来源依据 | 建议程度 | 具体问题 | 建议补充或改写范围 |
|---|---|---|---|---|---|
| 027 | [bsp/bsp_tools.c:39](../../../bsp/bsp_tools.c#L39) | 原有沿用；位置 `6c2f74b` | 一般 | 已有任务循环说明，但超过32个注册后的持续报错循环未解释，无文件用途说明。 | 补回调任务工具的用途及注册数量到上限时的行为。 |
| 028 | [bsp/bsp_tools.h:5](../../../bsp/bsp_tools.h#L5) | 原有沿用；位置 `6c2f74b` | 一般 | 说明只写收到信号才唤醒，遗漏任务启动后先调用一次回调；未说明最多32个注册及超出后的行为。 | 按实际执行顺序补回调首次调用和使用限制。 |
| 039 | [bsp/flash/bsp_flash.c:7](../../../bsp/flash/bsp_flash.c#L7) | 原有沿用；位置 `6c2f74b` | 一般 | 同一公开函数在源文件重复英文和中文接口说明，文件用途及写入循环停止条件缺少集中说明。 | 接口说明归头文件；源文件保留地址选扇区、写入停止和失败返回的原因。 |
| 040 | [bsp/flash/bsp_flash.h:38](../../../bsp/flash/bsp_flash.h#L38) | 原有沿用；位置 `6c2f74b` | 重点 | 接口多为英文，len 的 data num/page num 未明确字节、32位数据个数或扇区数量，地址使用条件未说明。 | 补当前用途和适用芯片提示，逐项明确长度单位、地址范围及返回值；不改地址常量。 |
| 041 | [bsp/gpio/bsp_gpio.c:1](../../../bsp/gpio/bsp_gpio.c#L1) | 原有沿用；位置 `6c2f74b` | 一般 | 有完整 EXTI 回调解释，但文件开头未交代 GPIO 注册表与中断查找的关系。 | 补简短文件用途及注册后如何触发模块回调的说明。 |
| 042 | [bsp/gpio/bsp_gpio.h:55](../../../bsp/gpio/bsp_gpio.h#L55) | 原有沿用；位置 `6c2f74b` | 一般 | 多个 @param 为空，返回值仅重复类型；GPIO_PIN 宏说明仍指向 F4 头文件。 | 补有效实例前提、引脚编号和读取结果，校正本工程引用。 |
| 043 | [bsp/iic/bsp_iic.c:106](../../../bsp/iic/bsp_iic.c#L106) | 原有沿用；位置 `6c2f74b` | 一般 | 无文件用途说明；寄存器读取/写入、地址宽度选择没有简短分支说明。 | 补 I²C 注册到传输的关系，说明寄存器地址宽度和完成回调的用途。 |
| 069 | [modules/TFminiPlus/tfminiplus.c:7](../../../modules/TFminiPlus/tfminiplus.c#L7) | 原有沿用；位置 `6c2f74b` | 一般 | 文件完全没有用途和步骤说明，距离读取时发送命令、接收数据和组合字段的关系不清楚。 | 简述测距流程及初始化延时的用途，接口用法集中在头文件。 |
| 070 | [modules/TFminiPlus/tfminiplus.h:16](../../../modules/TFminiPlus/tfminiplus.h#L16) | 原有沿用；位置 `6c2f74b` | 重点 | 注册和测距入口无注释，字段注释只写取自哪些字节，未解释距离单位和调用条件。 | 补实际数据含义、I²C 初始化前提、返回值与现有错误处理能力。 |
| 081 | [modules/algorithm/controller.h:7](../../../modules/algorithm/controller.h#L7) | 原有沿用；位置 `6c2f74b` | 一般 | 文件用途 @brief 留空；配置段多为英文标题，最大输出、积分上限及滤波时间参数缺简明含义。 | 补中文用途、配置字段和有效实例前提，保留已有 PID 接口说明。 |
| 082 | [modules/algorithm/crc16.c:9（现统一CRC入口）](../../../modules/algorithm/crc.c) | 原有沿用；位置 `6c2f74b` | 一般 | 中文句子有明显翻译断裂；函数原型和接口说明重复放在源文件，查表初始化两层循环的用途不直观。 | 改成自然中文，说明首次生成查表及逐字节更新，完整接口移到头文件。 |
| 083 | [modules/algorithm/crc16.h:9（现统一CRC入口）](../../../modules/algorithm/crc.h) | 原有沿用；位置 `6c2f74b` | 一般 | 所有公开 CRC16 入口无注释，未说明初值、字节长度、空数据和逐字节更新用法。 | 补中文文件用途与 Doxygen，说明两种入口当前使用的初值。 |
| 084 | [modules/algorithm/crc8.c:6（现统一CRC入口）](../../../modules/algorithm/crc.c) | 原有沿用；位置 `6c2f74b` | 一般 | 表用途说明仅英文且只谈 SHT75，未交代本工程通用 CRC8 用途；中文接口说明语序生硬。 | 简述查表计算用途，循环前说明逐字节累计即可。 |
| 085 | [modules/algorithm/crc8.h:8（现统一CRC入口）](../../../modules/algorithm/crc.h) | 原有沿用；位置 `6c2f74b` | 一般 | CRC8 两个公开入口无注释，未说明起始值、长度单位及空指针返回结果。 | 补简短中文接口说明与连续更新用法。 |
| 090 | [modules/bluetooth/HC05.c:1](../../../modules/bluetooth/HC05.c#L1) | 原有沿用；位置 `6c2f74b` | 一般 | 无文件职责说明；接收只接受固定帧头帧尾、发送添加帧头帧尾的结构未在文件开头说明。 | 补蓝牙固定帧用途，重要循环说明复制哪些数据，接口注意事项归头文件。 |
| 091 | [modules/bluetooth/HC05.h:18](../../../modules/bluetooth/HC05.h#L18) | 原有沿用；位置 `6c2f74b` | 一般 | 入口仅普通一句话，缺参数、初始化结果、长度单位和异步发送期间再次调用的条件。 | 补简短中文 Doxygen 及固定数据长度说明。 |
| 097 | [modules/imu/BMI088Middleware.c:1](../../../modules/imu/BMI088Middleware.c#L1) | 原有沿用；位置 `6c2f74b` | 一般 | 旧 BMI088 适配文件没有用途或历史保留说明，无法区分它和 modules/BMI088 的现行驱动；单字节收发前提未说明。 | 补旧适配层用途及 SPI 指针需先设置的前提，暂不删除或改实现。 |
| 098 | [modules/imu/BMI088Middleware.h:18](../../../modules/imu/BMI088Middleware.h#L18) | 原有沿用；位置 `6c2f74b` | 一般 | 公开片选与字节交换入口无注释，BMI088_SPI 由谁设置、输入输出和等待方式不清楚。 | 说明旧接口用途、先设置 SPI 地址和字节交换结果。 |
| 099 | [modules/imu/BMI088reg.h:1](../../../modules/imu/BMI088reg.h#L1) | 原有沿用；位置 `6c2f74b` | 一般 | 旧寄存器表无文件职责和使用范围，部分分组仅英文，与现行 bmi088_regNdef.h 容易混淆。 | 补保留用途及对应驱动提示，不逐个宏扩写。 |
| 121 | [modules/master_machine/seasky_protocol.c:122](../../../modules/master_machine/seasky_protocol.c#L122) | 原有沿用；位置 `6c2f74b` | 一般 | 解析中帧头/整帧检查失败后的返回及输出保留缺说明；文件用途未提示它与 SP 视觉协议的关系。 | 说明保留的协议用途和两次检查的作用，接口条件归头文件。 |
| 122 | [modules/master_machine/seasky_protocol.h:24](../../../modules/master_machine/seasky_protocol.h#L24) | 原有沿用；位置 `6c2f74b` | 重点 | 参数只有行尾短注释，float_length 的个数单位、缓冲区容量、解析失败返回0和输出保留未解释。 | 补中文 Doxygen、调用方准备的数据空间和失败结果。 |
| 132 | [modules/motor/DMmotor/dmmotor.c:129](../../../modules/motor/DMmotor/dmmotor.c#L129) | 原有沿用；位置 `6c2f74b` | 一般 | 无文件职责，单电机持续发送循环和停止时只发送零力矩的选择缺少说明；部分内部函数无用途说明。 | 说明当前只实现力矩控制及每轮收发目的，保留停止后仍收反馈的原因。 |
| 133 | [modules/motor/DMmotor/dmmotor.h:65](../../../modules/motor/DMmotor/dmmotor.h#L65) | 原有沿用；位置 `6c2f74b` | 重点 | 公开接口无注释，位置/速度/力矩单位、允许范围和每电机任务创建顺序未解释。 | 补中文 Doxygen，说明当前控制能力、参考值单位和编码器置零行为。 |
| 139 | [modules/motor/LKmotor/LK9025.h:96](../../../modules/motor/LKmotor/LK9025.h#L96) | 原有沿用；位置 `6c2f74b` | 一般 | 在线接口无说明；停止/使能参数为空，参考值单位未结合角度/速度/电流控制说明。 | 补中文 Doxygen、有效实例前提和在线返回值。 |
| 144 | [modules/motor/motor_task.h:19](../../../modules/motor/motor_task.h#L19) | 原有沿用；位置 `6c2f74b` | 一般 | @note 只有“好无语”等感想；接口说明未涵盖当前还承担 CAN/UART/VCP 接收派发。 | 换成实际职责和调用顺序，保留1kHz条件。 |
| 145 | [modules/motor/servo_motor/servo_motor.c:9](../../../modules/motor/servo_motor/servo_motor.c#L9) | 原有沿用；位置 `6c2f74b` | 一般 | 文件用途只写“第二版”；总线/PWM 两类分支、反馈循环及命令数组的作用未解释。 | 补舵机类型对应流程，按实际实现说明 PWM 路径直接使用占空比。 |
| 149 | [modules/oled/oled.c:333](../../../modules/oled/oled.c#L333) | 原有沿用；位置 `6c2f74b` | 一般 | 有较完整函数说明，但缺文件整体用途；字符取位、换行和逐页刷新循环缺少简明串联，部分指令说明仅英文。 | 说明先改内存图像再刷新显示，解释字体循环和页刷新。 |
| 150 | [modules/oled/oled.h:122](../../../modules/oled/oled.h#L122) | 原有沿用；位置 `6c2f74b` | 一般 | row 写“开始列”、col 写“开始行”，与绘制实现相反；文件 @brief 只写待重构，未交代当前作用。 | 校正行列含义，说明字符坐标与像素坐标、缓冲绘制和刷新关系。 |
| 151 | [modules/oled/oledfont.h:4](../../../modules/oled/oledfont.h#L4) | 原有沿用；位置 `6c2f74b` | 一般 | 版权与字库用途保留完整，但总体用途仅英文，字符起点、位序及 LOGO 数据读取方式缺中文说明。 | 保留 DJI 版权，补简短中文字库排列说明，不逐字节扩写。 |
| 154 | [modules/referee/crc_ref.c:115（现统一CRC入口）](../../../modules/algorithm/crc.c) | 原有沿用；位置 `6c2f74b` | 一般 | Append_CRC8/16 的注释写输出 True/False，但函数返回 void、直接改数据尾部；多数说明仅英文。 | 校正追加校验值与检查校验值的区别，用中文说明逐字节循环。 |
| 155 | [modules/referee/crc_ref.h:11（现统一CRC入口）](../../../modules/algorithm/crc.h) | 原有沿用；位置 `6c2f74b` | 重点 | 各公开函数无参数和返回说明，dwLength 是否包含校验字节、会改哪些字节、空指针行为未解释。 | 补中文 Doxygen、长度单位、初值和输出修改方式。 |
| 156 | [modules/referee/referee_UI.c:4](../../../modules/referee/referee_UI.c#L4) | 原有沿用；位置 `6c2f74b` | 一般 | 文件用途留空；长参数说明多在 .c 中，图形推送数量选择和无效数量的默认处理未解释。 | 接口说明集中到头文件，源文件说明组包、序号和推送循环。 |
| 157 | [modules/referee/referee_UI.h:41](../../../modules/referee/referee_UI.h#L41) | 原有沿用；位置 `6c2f74b` | 重点 | 所有公开绘图/发送接口无用法说明，图形名三字节、坐标单位、可变参数类型和仅支持1/2/5/7个图形等条件未交代。 | 补中文 Doxygen 和调用前准备的数据。 |

## 空文件：9 份用途待确认

这些文件没有可执行代码或接口。本轮不编造功能说明，也不要求补函数参数；若继续作为占位保留，建议只写一句当前用途和未实现状态。

| 序号 | 文件 | 来源依据 | 当前情况 | 建议 |
|---|---|---|---|---|
| 001 | [application/chassis/balance.h（空）](../../../application/chassis/balance.h) | 原有沿用；基线已存在 | 空文件，无法从文件内了解预留用途。 | 确认是否继续保留；保留时补一句用途。 |
| 006 | [application/chassis/mecanum.h（空）](../../../application/chassis/mecanum.h) | 原有沿用；基线已存在 | 空文件，未说明麦轮底盘预留内容。 | 确认保留用途后补简短占位说明。 |
| 007 | [application/chassis/steering.h（空）](../../../application/chassis/steering.h) | 原有沿用；基线已存在 | 空文件，未说明舵轮底盘预留内容。 | 确认保留用途后补简短占位说明。 |
| 147 | [modules/motor/step_motor/step_motor.c（空）](../../../modules/motor/step_motor/step_motor.c) | 原有沿用；基线已存在 | 空文件，未说明步进电机预留内容。 | 确认保留用途后补一句占位说明。 |
| 148 | [modules/motor/step_motor/step_motor.h（空）](../../../modules/motor/step_motor/step_motor.h) | 原有沿用；基线已存在 | 空文件，未说明步进电机接口是否已有实现。 | 确认保留用途后补一句占位说明。 |
| 171 | [modules/standard_cmd/std_cmd.c（空）](../../../modules/standard_cmd/std_cmd.c) | 原有沿用；基线已存在 | 空文件，未说明标准命令模块预留用途。 | 确认保留用途后补简短占位说明。 |
| 172 | [modules/standard_cmd/std_cmd.h（空）](../../../modules/standard_cmd/std_cmd.h) | 原有沿用；基线已存在 | 空文件，没有可审查的标准命令接口。 | 确认是否保留；保留时说明尚未实现。 |
| 177 | [modules/unicomm/unicomm.c（空）](../../../modules/unicomm/unicomm.c) | 原有沿用；基线已存在 | 空文件，未说明统一通信模块预留用途。 | 确认保留用途后补简短占位说明。 |
| 178 | [modules/unicomm/unicomm.h（空）](../../../modules/unicomm/unicomm.h) | 原有沿用；基线已存在 | 空文件，没有可审查的统一通信接口。 | 确认是否保留；保留时说明尚未实现。 |

## 暂未发现明确注释问题：5 份

这里表示本轮注释检查没有需要列入修改清单的具体问题，不代替功能验证。DR 文件仅评价占位说明是否清楚。

| 序号 | 文件与位置 | 来源依据 | 结论 |
|---|---|---|---|
| 071 | [modules/alarm/alarm_def.h:4](../../../modules/alarm/alarm_def.h#L4) | 后续移入 `ef9cab8` | 报警通用定义、等级和开关分组已简明说明，无函数或复杂流程。 |
| 096 | [modules/general_def.h:4](../../../modules/general_def.h#L4) | 混合修改；位置 `6c2f74b`；最近 `47133e8` | 通用数值换算用途明确，各换算因子旁有方向或计算式，无复杂流程。 |
| 123 | [modules/master_machine/vision_control.c:1](../../../modules/master_machine/vision_control.c#L1) | 后续新增 `d0810e4`；位置 `89e61b9` | 文件职责、头文件分工、失败清零、单位换算及重要选择已按认可样例说明。 |
| 124 | [modules/master_machine/vision_control.h:1](../../../modules/master_machine/vision_control.h#L1) | 后续新增 `d0810e4`；位置 `89e61b9` | 文件、数据定义和公开接口已说明参数、单位、时间要求及失败输出，语言简明。 |
| 134 | [modules/motor/DRmotor/drmotor.c:1](../../../modules/motor/DRmotor/drmotor.c#L1) | 后续移入 `dc6b5c6` | 纯注释占位文件已明确来源、尚未实现和适配条件，无可执行函数。 |

## 核对依据与后续选择

对容易误导读者的条目回看了对应实现。例如，CAN 的等待说明对照 `CANTransmit` 中忽略 timeout 并直接检查发送空间的代码；INS 初始化说明对照 `StartINSTASK` 的实际调用；PID 的“返回空”对照 float 返回类型；消息发布的返回数量说明对照成功固定返回1的实现；WS2812 编码说明对照每个颜色位保存一个字节的循环。按需标定的旧中键说明对照当前 LCD 的左右长按处理。

本报告列的是每份文件已经找到的具体问题及建议范围，不承诺该文件内每句注释只有这些问题。将来获准修改时仍需结合整份文件和调用处核实，不把尚未实现的功能写进注释。

本轮核对文件清单与 Git 跟踪文件一致，共178份且无重复；检查行号、来源提交和本地链接，并通过 `git diff --check`。与检查起点相比，业务源码无差异。本轮不涉及运行行为，未运行固件编译、主机回归或上板操作。

你可以按序号或路径选择一份、一对头文件与源文件或一组模块。收到选择后再补注释；本轮不批量改写源码，也不把164份文件自动登记成164个开发任务。如何核对注释见[学习笔记](../../学习笔记/22-失联安全与有界等待.md#先检查注释再决定修改范围mc02-0107)。
