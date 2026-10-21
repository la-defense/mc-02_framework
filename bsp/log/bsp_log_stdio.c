/** @file bsp_log_stdio.c
 * @brief 禁止固件标准输入输出绕过类型化日志和硬件安全诊断。
 * @note stdout/stderr、stdin 均不连接 UART；日志只能通过 BSP_LOG_EVENT_* 提交。
 */
#include <errno.h>
#include <stdio.h>

/** @brief 关闭标准输入；有效 stdin 请求明确返回不支持。
 * @param[in] file 文件描述符，只接受0。
 * @param[out] buffer 接收地址，正长度时不可为NULL。
 * @param[in] length 字节数；负值返回EINVAL，零长度返回0。
 * @return 有效读取返回ENOSYS错误；无效描述符返回EBADF。
 */
int _read(int file, char *buffer, int length)
{
    if (file != 0)
    {
        errno = EBADF;
        return -1;
    }
    if (length < 0 || (length > 0 && buffer == NULL))
    {
        errno = EINVAL;
        return -1;
    }
    if (length == 0)
        return 0;

    errno = ENOSYS;
    return -1;
}

/** @brief 关闭标准输出转发；正长度请求不写入任何串口。
 * @param[in] file 文件描述符，只接受1或2。
 * @param[in] buffer 待写数据；正长度时不可为NULL。
 * @param[in] length 字节数；负值返回EINVAL，零长度返回0。
 * @return 正长度返回ENOSYS错误；无效描述符返回EBADF。
 */
int _write(int file, char *buffer, int length)
{
    if (file != 1 && file != 2)
    {
        errno = EBADF;
        return -1;
    }
    if (length < 0 || (length > 0 && buffer == NULL))
    {
        errno = EINVAL;
        return -1;
    }
    if (length == 0)
        return 0;

    errno = ENOSYS;
    return -1;
}

/** @brief 禁止 fputc 直接发送字符。
 * @param[in] character 要写入的字符，本实现不会读取其值。
 * @param[in] stream 仅接受stdout或stderr；其他流返回EINVAL。
 * @return 始终返回EOF并设置ENOSYS；空流指针返回EOF并设置EINVAL。
 */
int fputc(int character, FILE *stream)
{
    (void)character;
    if (stream != stdout && stream != stderr)
    {
        errno = EINVAL;
        return EOF;
    }
    errno = ENOSYS;
    return EOF;
}
