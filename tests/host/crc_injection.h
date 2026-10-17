/** @file crc_injection.h
 * @brief 主机测试的 CRC 失败注入：仅测试链接使用，不编入固件。
 */
#ifndef MC02_CRC_INJECTION_H
#define MC02_CRC_INJECTION_H
/** @brief -1正常运行；0让下一次计算失败；正数表示先成功多少次再失败一次。 */
extern int crc_fail_after;
#endif
