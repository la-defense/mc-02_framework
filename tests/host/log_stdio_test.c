/** @file log_stdio_test.c
 * @brief 验证标准输入输出被关闭，且失败路径不调用UART或日志后端。
 * @note 直接链接生产 stdio 适配；所有检查在Release下仍生效。
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_check.h"

int _read(int file, char *buffer, int length);
int _write(int file, char *buffer, int length);

int main(void)
{
    char byte = 'x';

    TEST_CHECK(_write(1, &byte, 0) == 0);
    TEST_CHECK(_write(2, &byte, 1) == -1 && errno == ENOSYS);
    TEST_CHECK(_write(0, &byte, 1) == -1 && errno == EBADF);
    TEST_CHECK(_write(1, NULL, 1) == -1 && errno == EINVAL);
    TEST_CHECK(_write(1, &byte, -1) == -1 && errno == EINVAL);

    TEST_CHECK(_read(0, &byte, 0) == 0);
    TEST_CHECK(_read(0, &byte, 1) == -1 && errno == ENOSYS);
    TEST_CHECK(_read(1, &byte, 1) == -1 && errno == EBADF);
    TEST_CHECK(_read(0, NULL, 1) == -1 && errno == EINVAL);

    FILE *volatile invalid_stream = NULL;
    TEST_CHECK(fputc('x', stdout) == EOF && errno == ENOSYS);
    TEST_CHECK(fputc('x', invalid_stream) == EOF && errno == EINVAL);
    return EXIT_SUCCESS;
}
