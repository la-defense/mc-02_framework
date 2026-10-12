#ifndef MC02_HOST_TEST_CHECK_H
#define MC02_HOST_TEST_CHECK_H

#include <stdio.h>
#include <stdlib.h>

#define TEST_CHECK(expression)                                                   \
    do                                                                          \
    {                                                                           \
        if (!(expression))                                                      \
        {                                                                       \
            (void)fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,     \
                          __LINE__, #expression);                               \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

#endif
