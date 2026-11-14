# MC02-0054：主线整合前的验收记录

以下保留原 TODO 的历史数字和限制，只对应当时版本。494 与后续 495 次回归按原记录先后保留。任务状态仅在 TODO 维护。

建立车型配置、功能依赖与编译选择（R01）。验收：每兵种一套当前配置，首台单板哨兵四轮全向轮；声明五种底盘并按配置筛选源码。未实现车型／DR／步进／半舵选用时明确报错，无默认车型回退；核心失败失能，辅助按依赖降级，非法配置和编译组合回归通过。软件 Debug／Release／ASan／UBSan 各 494/494 通过，禁热 ARM 固件和无 LCD 构建通过，生产哨兵按 APP-10／MC02-0058 条件拒绝；探针已识别为 Horco CMSIS-DAP v2，但 OpenOCD 在 400 kHz 连接时 `CMD_INFO` 失败，尚未读取或备份 Flash；待探针恢复后完成获批裸板备份、烧录、验证及观察。详见[范围规格](../../../../docs/workflow/specs/robomaster-framework-evolution.md)。追加 B2／B6／S1：平台／板卡／RTOS 与车型分离；Seasky 退出构建并保留参考，删除旧 BMI088Middleware.c/.h、BMI088reg.h 及引用，保留现行 BMI088 和 Git 来源；接口解耦关联0086／0087，不恢复 R27。 2026-11-01复核补充：WSL Debug、Release、ASan/UBSan均495/495通过，禁热与无LCD ARM构建通过；探针仍报CMSIS-DAP CMD_INFO失败，未读写Flash或烧录，裸板验收待探针恢复。
