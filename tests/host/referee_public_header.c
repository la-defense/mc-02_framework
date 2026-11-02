/** @file referee_public_header.c
 * @brief 检查裁判公开接口可以只使用框架头编译。
 */
#include "../../modules/referee/rm_referee.h"

RefereeStatus_t (*const referee_init_signature)(PlatformUartPort_t, const RefereeConfig_t *) = RefereeInit;
