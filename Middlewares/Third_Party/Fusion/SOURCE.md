# Fusion 来源与使用范围

- 上游：<https://github.com/xioTechnologies/Fusion>
- 固定提交：`a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`
- 许可：MIT，原文见本目录 `LICENSE.md`。
- 本工程仅为 INS 的可选姿态后端保留必要 AHRS 实现；不使用上游构建脚本或其他传感器算法。
- 适配器把陀螺仪 rad/s 转成 deg/s，把加速度 m/s² 转成 g，并将坐标约定设为 NWU。
- 本目录文件来自上述提交，未作源码修改；校验值由 `sha256sum` 复核。

| 文件 | SHA-256 |
|---|---|
| `FusionAhrs.c` | `611678dbaa531db3b360cf3f3097a52cd60d7cf2f93be0753748d8674b9cb5fb` |
| `FusionAhrs.h` | `fd726d91ece9a5a0074151e5b777aa9578fe3f365a03b94be7d62e4084d23200` |
| `FusionMath.h` | `c7f8a14111418ccc2e8c81ab7718baa7c2b3a6f87756954a2922579405d056a0` |
| `FusionInline.h` | `4911c37b8876066054345307f91e6ad015c3b261b036fab619dcd090b45b7dd1` |
| `FusionConfig.h` | `1042ea5f363551d774a339e53c7cc4ce4cb1a5b90eb8fdda19d5ac2e8009ccd6` |
| `FusionConvention.h` | `781f26fb04d3d0bee5070c5e4e13f6f6fb953a57368e83a6b9bc87da3e266a3c` |
| `LICENSE.md` | `deea99a9e5920dc1cb5bfd7730fba6ca3fdab691bde7760aa1642e65c650ccca` |

主机测试可同时编译两个后端；固件使用`MC02_IMU_BACKEND=quaternion_ekf|fusion`选择一个后端。未选择Fusion时，其C实现不进入固件源码清单。
