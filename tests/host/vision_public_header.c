/** @file vision_public_header.c
 * @brief 检查上位机通信公开接口不依赖芯片或 RTOS 头文件。
 */
#include "../../modules/master_machine/master_process.h"

Vision_Recv_s *(*const vision_init_signature)(PlatformUartPort_t) = VisionInit;
