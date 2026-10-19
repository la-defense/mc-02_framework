# 算法模块入口

<p align='right'>neozng1@hnu.edu.cn</p>

本目录提供纯计算入口。使用前阅读对应头文件的参数、容量、生命周期和失败处理；任务状态统一见 [TODO](../../.Doc/TODO.md)。

| 用途 | 头文件与调用条件 |
|---|---|
| PID | [controller.h](controller.h)：调用者保存实例，初始化后传入秒单位的实测dt |
| CRC | [crc.h](crc.h)：五种固定配置，整块计算或独立增量上下文 |
| KF | [kalman_filter.h](kalman_filter.h)：调用者长期保存工作区，结果区分校正、仅预测和无效 |
| 姿态EKF | [QuaternionEKF.h](QuaternionEKF.h)：当前INS使用的单IMU入口 |
| RLS | [RLS.h](RLS.h)：固定维度和独立工作区，成功后才发布参数与协方差 |
| 标量／角度 | [math_utils.h](math_utils.h)：限幅、开方、舍入及半开区间绕回，返回状态和独立结果 |
| 三维向量 | [vector3.h](vector3.h)：模长、单位方向、点积和叉积，允许输出与输入重叠 |
| 公共滤波 | [sample_filter.h](sample_filter.h)：独立实例的一阶低通、平均、二阶低通和陷波；显式输入实测dt，异常后由新样本重建 |
| 矩阵求逆 | [dsp_matrix.h](dsp_matrix.h)：用独立工作区隔开厂商求逆会改写的输入，失败保留旧输出 |

MC02-0076移除了旧user_lib混合入口、无人使用的矩阵宏和动态分配MatInit。BMI088在驱动内部申请并清零自身实例；数值工具不负责分配。

MC02-0077的SampleFilterInit／Reset／Update／GetState由调用者保存实例，平均种类另提供独占窗口，运行不分配。新实例首份样本及异常后恢复使用常值历史，返回FILTER_REBUILT；旧MovingAverageF32仍保留零填充权重，HT04行为不变。电机按实例加载参数和迁移由MC02-0017跟踪，当前控制参数没有改变。用法、响应图及验证范围见[公共滤波说明](../../docs/学习笔记/24-嵌入式固件问题定位.md#四类公共滤波和异常后重建mc02-0077)。

测试通过公开生产入口执行；硬件替身只替代SPI、CAN、时钟等外部条件。规则和例子见[学习笔记24](../../docs/学习笔记/24-嵌入式固件问题定位.md#纯数值入口的范围重叠和失败输出mc02-0076bspmisc-03)。

