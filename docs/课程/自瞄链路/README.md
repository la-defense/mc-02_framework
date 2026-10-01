# 自瞄链路课程

这组课程围绕一条可验证的链路：MC-02 用 SP 帧传状态，上位机做识别与规划，再将禁止开火的瞄准结果发回。示例全部使用主机模拟数据，不连接执行器。

## 学习顺序

1. [SP 帧与 CRC](lessons/0001-sp-frame-crc.html) — 固定长度、字段布局、CRC 校验与异常帧。
2. [USB CDC 收发](lessons/0002-usb-cdc.html) — 持久发送缓冲、忙状态和完成回调。
3. [视觉超时与撤销控制](lessons/0003-vision-timeout-revoke.html) — 目标失效后将旧控制撤销为中立帧。
4. [Windows–WSL 相机及串口桥接](lessons/0004-windows-wsl-bridge.html) — 将 Windows SDK 采集的数据送进 Linux 识别进程。

每课都链接到一组 [主机练习](exercises/README.md) 和一份 [速查页](reference/)。课程写出不代表掌握；只有用户回答练习或解释关键行为后，才记录学习记录。

## 开发者入口

- 学习目标：[MISSION.md](MISSION.md)
- 可信来源：[RESOURCES.md](RESOURCES.md)
- 讨论偏好：[NOTES.md](NOTES.md)
- 工作区资源：[assets/lesson.css](assets/lesson.css)
- 验证：`python tools/workflow/check_exercises.py --build`
