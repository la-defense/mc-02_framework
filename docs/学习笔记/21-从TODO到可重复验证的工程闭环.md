# 从 TODO 到可重复验证的工程闭环

本工程有固件和视觉两个 Git 仓库。各自的 TODO 是任务状态的唯一来源；规格、代码、测试和 PR 通过稳定任务编号相互链接。跨仓库工作在两个 TODO 中各有一条对应任务，并在两个仓库分别提交。

## 为什么先找一个可运行的检查

修复一个问题时，先找到能触发并识别它的最小检查，再改代码。MC-02 的主机测试直接调用 `VisionProtocolFindCommandFrame` 和 `VisionControlInputIsUsable`，可以在没有 STM32、USB 或云台的情况下检查帧长、CRC、超时和非有限数值。视觉的 `sp_vision_protocol_test` 检查 43/29 字节帧、CRC、禁火瞄准帧及无目标、失联、无效规划时生成的中立帧。

这些测试覆盖的是纯 C/C++ 逻辑边界；它们不证明硬件收发、相机时序或机械限位。那些行为仍需独立的人工 HIL 验收。

## CMake 和 CTest 如何连起代码与证据

每组主机测试由 CMake 描述：哪些源文件组成测试程序、需要哪些头文件、测试的名字是什么。构建目录放在仓库之外或被忽略的 `build/` 中，避免生成物混入提交。

`ctest` 根据构建目录中的测试清单运行程序并汇总退出状态。CI 执行相同的配置、构建和测试步骤，因此开发机和 GitHub Actions 共享同一套可复现检查。

视觉协议测试必须在 Release 配置下也保留检查。标准 C `assert`/C++ `assert` 会在定义 `NDEBUG` 时被移除；所以测试使用 `CHECK`，条件失败时打印表达式并以非零退出，而不依赖构建类型。

```sh
cmake -S tests/host -B build/host-tests -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests --parallel 2
ctest --test-dir build/host-tests --output-on-failure
```

视觉仓库的离线 CPU 测试用 CTest 运行固定长度的演示视频片段，不打开 GUI，也不加载工业相机 SDK。回放汇总报告处理帧数、有目标帧数，以及同一批有目标帧中产生有限瞄准输出的帧数。

## 提交和审查顺序

一项任务在任务分支上完成并提交后，再审查从分支基线到提交的完整差异。审查分成 Standards 和 Spec 两条检查：前者关注项目约定，后者逐条对照 TODO 验收条件。审查发现的问题修复后另作提交，再审查后续提交。

任务结束时，TODO 状态、提交 SHA、测试结果和学习笔记应能互相追溯。课程或文档写出来只表示有可复习材料，不表示学习者已经掌握；学习记录只在学习者通过回答或练习展示理解后增加。

## 相关实现

- [MC-02 主机安全测试](../../tests/host/CMakeLists.txt)
- [MC-02 固件与主机 CI](../../.github/workflows/c-cpp.yml)
- [课程与 C/C++ 练习](../课程/自瞄链路/README.md)
- [视觉协议测试和 CPU 构建](../../../sp_vision_25-upstream/CMakeLists.txt)
- [工程工作流规格](../workflow/specs/engineering-learning-workflow.md)
