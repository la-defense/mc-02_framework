# 主机练习

每个练习都有 `problem / solution / explainer` 三个目录。problem 是学员起点，solution 是参考实现与可执行测试，explainer 说明代码所模拟的边界。所有数据为内存中的模拟帧或状态，不调用真实硬件。

| 顺序 | 主题 | 课程 |
|---|---|---|
| 01 | SP 帧长度、CRC 和损坏帧 | [第 1 课](../lessons/0001-sp-frame-crc.html) |
| 02 | CDC 持久缓冲和 BUSY 状态 | [第 2 课](../lessons/0002-usb-cdc.html) |
| 03 | 视觉超时、有限值和中立撤销 | [第 3 课](../lessons/0003-vision-timeout-revoke.html) |
| 04 | 相机/串口桥接健康度 | [第 4 课](../lessons/0004-windows-wsl-bridge.html) |

运行 `python tools/workflow/check_exercises.py --build` 会检查本地链接、配置 CMake、构建练习并执行 CTest。
