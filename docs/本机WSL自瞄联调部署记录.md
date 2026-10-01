# 本机 WSL 自瞄联调部署记录

日期：2026-10-01
工程：MC-02 STM32H723 固件 + TongjiSuperPower/sp_vision_25 上位机
状态：WSL 2/Ubuntu 22.04.5 已部署；官方上位机 CPU 构建成功，演示自瞄及 Windows MVS 相机桥输入已实测。MC-02 USB CDC 直通并持续解析状态帧；重插 CMSIS-DAP 后已通过 OpenOCD/SWD 复核 HIL 后的 RX、CRC、TX 与 USB busy 计数。

## 验收边界

MC-02 当前是裸板，没有云台电机。本阶段只验证上位机生成、发送瞄准指令及下位机解析，不驱动云台或发射。测试控制帧使用 `mode=1`（瞄准控制、不允许开火）；实际联调程序仍需强制禁止开火，并在无目标、断连和退出时发中立帧。

## Windows 环境准备

- Windows 10 Pro for Workstations，版本 10.0.19045；Lenovo XiaoXin 15 IRH9（83G1），CPU Intel i5-13420H，内存 16 GB。
- 当前检查时 HypervisorPresent=True，Windows 已检测到运行中的 hypervisor；WSL 与 Virtual Machine Platform 功能启用，Ubuntu 22.04.5 LTS 以 WSL 2 运行。
- Windows 可选功能 Microsoft-Windows-Subsystem-Linux、VirtualMachinePlatform 已启用；wsl --update 成功，默认 WSL 版本设为 2。
- 已安装 `usbipd-win 5.3.0`，服务自动启动。
- USB 设备：海康/MVU3V 相机 `2BDF:0001`（BUSID `2-13`）；MC-02 ST 虚拟串口 `0483:5740`（COM14，BUSID `3-3`）；Horco CMSIS-DAP `FAED:4870`（COM9，BUSID `3-4`）。BUSID 可能随重插变化，执行 usbipd 命令前应重新运行 `usbipd list`。
- 现有个人副本位于 `D:\RoboMasterRelated\OpenSourseProject\sp_vision_25-main`，未修改。官方仓库在 WSL 中的 clone commit 为 `bd9f5e798fa3c6dd3b483ae6627796afb41c608d`。

## 已实施的 MC-02 修复

- USB CDC 发送入口检查空指针、零长度、最大长度、设备配置状态和 CDC class 指针，避免 USB 未枚举/拔出时解引用空指针。
- USB CDC 提交前复制到持久发送缓冲；CDC 忙时不覆盖在途数据并返回忙状态。上层统计成功提交数和忙丢帧数。
- 增加 29 字节上行帧、43 字节下行帧编译期断言；保护视觉帧快照和统计计数；视觉服务离线时清零旧接收命令和时间戳。
- 工程 Debug 构建成功，产物 `build-audit/Basic_Framework_MC02.elf`。RAM 使用 106392/131072 B（81.17%），Flash 使用 173808/786432 B（22.10%）。仍有既有 unused-variable 与 `volatile` 限定符告警。
- 使用 CMSIS-DAP/OpenOCD 对该 ELF 烧录并校验通过。刷写脚本最后因自身访问不存在的 `args.no_save_profile` 返回退出码 1，但日志明确报告烧录成功且校验通过。

## 已完成的通信检查

- Windows COM14 连续接收 602 个通过 CRC 的 43 字节 `SP` 状态帧（3 秒，199.4 Hz），CRC 错误 0。
- 通过 COM14 发送 50 个 29 字节 `SP` 瞄准帧，`mode=1`、CRC 使用多项式 `0x8408`、初值 `0xFFFF`，所有帧禁止开火。
- WSL HIL 前的 Windows 固件基线：CRC 错误 `0`、USB busy 丢帧 `204`、成功提交状态帧 `211`、接收指令 `50`；等待视觉离线超时后接收命令结构清零。HIL 后最新计数见下文。
- CMSIS-DAP 重插后，Windows OpenOCD 识别到 Horco v0.2，SWD DPIDR 为 `0x6ba02477`，可读写 MCU 调试端口。WSL HIL 前后计数：运行前 CRC=`0`、busy=`1,062,724`、状态帧提交=`32,148`、接收命令=`1,243`；运行后 CRC=`0`、busy=`1,063,036`、状态帧提交=`35,502`、接收命令=`1,545`。接收命令增加 `302`，与 268 个瞄准帧、32 个无目标中立帧和退出时两次中立帧相符；CRC 错误没有增加。
- 一次 300 帧 WSL HIL 的中段 SWD 采样中，USB busy 计数保持不变；状态帧提交数分别在约 1.8 秒增加 362、约 2.3 秒增加 459，约为 200 Hz。程序结束且 CDC 无读取者后，busy 计数开始增加，说明持续运行应保持上位机状态帧读取；这不影响 HIL 期间的命令解析。
- Windows COM14 独立监听 5.027 秒收到 1004 个 CRC 正确的 43 字节状态帧（199.73 Hz），坏帧 0；同期固件状态帧提交数增加 1006，USB busy 丢帧数不变。
- HIL 计数快照之后，长时间 OpenOCD 会话出现 CMSIS-DAP USB I/O 错误；关闭后短时重连成功，目标显示 running，但 RX=`0`、TX=`1`，说明 MCU 计数状态此后重新初始化，具体复位原因尚未确认。因此上述 HIL 前后数值是当次运行的已保存快照；后续读取建议使用短时 OpenOCD 会话，并单独排查复位来源。

## WSL 部署、构建与联调结果

- 使用 WSL 2 Ubuntu 22.04.5 LTS，默认 Linux 用户 mc02。官方仓库克隆在 /home/mc02/src/sp_vision_25，基线 commit bd9f5e798fa3c6dd3b483ae6627796afb41c608d；Windows staging 副本中准备的 overlay 已应用。现有个人副本 sp_vision_25-main 未修改。
- USB 设备节点由 root 持有，直接访问示例需 `sudo -E`。如果 mc02 的 sudo 密码不可用，可在 Windows 运行 `wsl -d Ubuntu-22.04 -u root -- passwd mc02` 设置新密码；不要把密码写入仓库或部署记录。
- 为绕过当前环境对 Ubuntu 镜像 HTTP 软件包下载的连接失败，将 /etc/apt/sources.list 的 Ubuntu 镜像地址切换为 HTTPS。安装 C++、CMake、Ninja、OpenCV、Ceres、Eigen、spdlog、yaml-cpp、nlohmann-json、libusb、socat；安装 OpenVINO 2024.6 CPU runtime。
- WSL CMake preset 成功配置并构建 standard_mpc、auto_aim_test、bareboard_hil。编译中发现并修复 tcp_frame_camera.cpp 缺少 OpenCV imgcodecs include。
- 离线 demo 联调运行 300 帧，MC-02 状态流保持在线。识别到目标的帧数为 268，规划产生 268 次有限瞄准输出，剩余帧发送中立命令，退出时再次发送中立帧。瞄准日志 fire=forced-off；云台没有电机连接，也没有开火。
- 初次直连 HIL 暴露 gimbal serial reader 使用零超时造成忙轮询、反复重开串口并与发送竞争。已在上位机暂存源和 WSL 工程中设置 20 ms 串口超时，连续 10 次读失败后重连；重建后再次运行 300 帧，无串口写失败。
- MC-02 CDC 0483:5740 通过 usbipd 直通 WSL，加载 cdc_acm 后出现 /dev/ttyACM0；上位机收到并 CRC 校验有效的状态帧，运行日志 mc02_online=true。HIL 前 Windows 端的 50 条禁火指令和 CRC=0 是历史基线，WSL HIL 后读取的最新计数见上文。
- 相机 2BDF:0001 经 usbipd 可在 WSL lsusb 中枚举，但随仓库提供的 Linux MVS SDK 反复返回 MV_CC_EnumDevices 0x80000006。随后将相机交还 Windows，通过已安装的 MVS 5.1.0 桥向 WSL 发送 JPEG；联调识别程序连续处理 100 帧，约 29 fps。此段实拍没有装甲板，程序发送中立命令，符合本阶段验收边界。
- WSL 通过 USB/IP 直连 MC-02 再次运行 300 帧：268 帧有效目标/有限瞄准输出、32 帧中立输出，状态流在线；所有瞄准日志均为 `fire=forced-off`。SWD 前后 RX 增加 302、CRC 错误维持 0，证明瞄准和中立控制帧均被下位机解析。USB busy 增量出现在程序退出后的无人读取阶段；HIL 活跃中段采样没有增长。
- OpenCV CommandLineParser 对联调参数需使用 --config-path=configs/demo.yaml 形式；用空格拆开选项和值会将视频路径和 YAML 路径误解析。WSL HIL 文档中的命令已改为等号写法。
- WSL 测试 IP 172.30.227.104，Windows NAT 网关 172.30.224.1；这些地址会随重启变化，启动桥接前需重新查询。桥接脚本仅接受传入的 --allow-peer 指定客户端地址。

### 还需完成

- 若需要 Linux 原生 MVS USB 取流，另行排查 SDK 返回值 `0x80000006`；当前 Windows MVS 桥已把相机画面送入 WSL 识别程序。
- 有装甲板的实拍、云台运动和发射尚未验收；当前测试没有连接电机或发射机构，本地只验证了识别、规划、禁火指令发送及下位机解析。
- 排查长时间 OpenOCD 轮询期间的 CMSIS-DAP USB I/O 错误及 MCU 计数重新初始化原因；HIL 计数快照已在错误出现前保存。

## 参考

- [官方上位机仓库](https://github.com/TongjiSuperPower/sp_vision_25)
- [Microsoft：手动安装 WSL](https://learn.microsoft.com/windows/wsl/install-manual)
- [Microsoft：将 USB 设备连接到 WSL](https://learn.microsoft.com/windows/wsl/connect-usb)
