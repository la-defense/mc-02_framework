/** @file dji_can_unregister_fake.c
 * @brief 静态 CAN 测试实例不分配资源，启动回退不释放内存。
 */
#include "bsp_can.h"

void CANUnregisterInstance(CANInstance *instance)
{
    (void)instance; // 这些测试的实例由固定数组提供。
}
