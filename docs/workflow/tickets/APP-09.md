# APP-09：云台模式与状态转换核对

## 范围

本项核对 `GimbalTask()` 的模式注释、生产调用和模块说明。保留电机控制公式、PID 参数、反馈选择及消息接口，不改变真实控制策略。

## 生产行为

云台命令是本地快照，最大保存50 ms。读取成功后按模式执行；读取失败时清空本地命令并进入停止分支。

| 条件 | 进入条件 | 对 yaw、pitch 的处理 |
|---|---|---|
| `GIMBAL_ZERO_FORCE` | 收到新鲜的零力模式，或命令读取失败后本地命令被清零 | 调用 `DJIMotorStop()`，参考值设为0 |
| `GIMBAL_GYRO_MODE` | 收到不超过50 ms的有效命令 | 使能两电机；角度环和速度环均设为 `OTHER_FEED`；yaw、pitch 度数乘 `DEGREE_2_RAD` 后设置参考 |
| `GIMBAL_FREE_MODE` | 收到不超过50 ms的有效命令 | 与 GYRO 相同；当前尚无单独的底盘跟随行为 |
| 未知模式 | 新鲜命令中的枚举值不在已知范围 | 与零力分支相同 |

`OTHER_FEED` 由 `ReadGimbalSignal()` 提供 INS 数据：yaw 位置为 `yaw_total_rad`、yaw 速度为 Z 轴陀螺仪；pitch 位置为 `pitch_rad`、pitch 速度为 X 轴陀螺仪。单位分别为 rad 和 rad/s，反馈配置最大年龄5 ms。

## 实现与验证

- 更新 `application/gimbal/gimbal.c`、`application/robot_def.h` 注释及 `application/gimbal/gimbal.md`，把模式名称与实际动作对齐。
- `tests/host/gimbal_mode_test.c` 直接编译生产 `GimbalTask()`，验证初始化配置、两种活动模式的电机动作和单位换算、零力／未知／失效命令停止路径，以及反馈读取内容。
- 主机替换消息中心、INS、电机动作和毫秒时钟，不连接电机；测试不证明 CAN 发送、真实转向或机械停止时间。

## 验收记录

- WSL GCC 11.4.0：主机 Debug 与 Release 均通过 **420/420** 项，包含 `production_gimbal_mode_state_transitions`。
- ARM GCC 10.3.1：`bench-debug`、`bench-rel`、`rcdiag-debug`、`rcdiag-rel`、`robot-single-debug`、`robot-single-rel`、`robot-chassis-debug`、`robot-chassis-rel`、`robot-gimbal-debug`、`robot-gimbal-rel` 十种禁热配置全部构建成功；其中 single 与 gimbal 配置实际编译更新后的 `application/gimbal/gimbal.c`。
- 配置阶段格式诊断正反例均通过（16项）；构建中未改控制参数和计算路径。此次仅添加 HAL tick 的显式头文件依赖并修订注释，因此没有单独做固件资源前后比较。
- `git diff --check` 通过。本项不连接硬件；真实电机响应与机械停止行为仍需相应台架验证。
