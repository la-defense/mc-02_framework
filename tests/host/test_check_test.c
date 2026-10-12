#include "test_check.h"

#include <string.h>

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--intentional-failure") == 0)
        TEST_CHECK(0 && "intentional failure control");

    TEST_CHECK(argc == 1);
    TEST_CHECK(1);
    return 0;
}
