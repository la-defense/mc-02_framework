# MC-02 与视觉工程的开发、教学工作流

## 问题

工程工作分布在 STM32 固件 MC-02 与 sp_vision_25 上位机两个仓库。原始 skills 将状态放在 `.scratch` 或外部 issue tracker；学习资料缺少与任务、代码提交和可运行练习的连结，硬件测试又需要严格隔离和可信 runner。

## 方案

- 每个仓库以自己的 TODO 作为唯一任务状态来源；规格、票据细节、决定图、PR 和课程只链接任务，不复制状态。
- tasks 使用分支和 PR；提交完整改动后进行 Standards 与 Spec 双轴代码审查。跨仓库任务在两个 TODO 中建立互相指向的条目，两边分别提交。
- Skills 只装在 MC-02。跨仓库工作从 MC-02 启动，再切换到视觉代码仓库。
- 日常新概念由 agent 结合当前代码简短解释并更新中文学习笔记。影响当前设计判断或用户表示不理解时，进入 teach 短课；掌握记录只能根据用户展示的理解新增。
- 主机课程模拟 SP 帧、CDC、视觉有效期和 Windows/WSL bridge 数据，不操作真实执行器；hardware HIL 必须由受限 Windows runner 在人工触发下针对输入的确切 SHA 运行。

## 验收

- 本地 agent 指引与适配 skills 将 TODO 作为任务状态唯一来源，并执行提交后 review 顺序。
- MC-02 有固件协议、CRC、异常帧和控制有效期主机测试；Vision 有无工业 SDK 的 CPU 配置、协议测试和 headless demo 回放。
- 四个离线教学主题均有 HTML 课、速查页和 C/C++ problem/solution/explainer 练习，链接检查与 CMake/CTest 可运行。
- HIL 报告使用固件和视觉精确 SHA，记录禁止开火的帧、下位机 RX/CRC 变化以及复位状态。
- 任务收尾指向已有或更新的学习笔记；没有用户理解证据时不写掌握记录。

## 状态位置

- MC-02：[MC02-0030 / MC02-0031 / R25-13 / HIL-10](../../../.Doc/TODO.md)
- sp_vision_25：[VIS25-0013 / VIS25-0014 / VIS25-0001–0012](https://github.com/la-defense/sp_vision_25/blob/85c27907725b58929505de69f05a6235cf217386/TODO.md)
