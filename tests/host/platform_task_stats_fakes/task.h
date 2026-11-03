#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "FreeRTOS.h"
#include <stdint.h>

typedef struct
{
    const char *pcTaskName;
    uint32_t ulRunTimeCounter;
    UBaseType_t uxCurrentPriority;
    uint32_t usStackHighWaterMark;
} TaskStatus_t;

UBaseType_t uxTaskGetNumberOfTasks(void);
UBaseType_t uxTaskGetSystemState(TaskStatus_t *records,
                                 UBaseType_t capacity,
                                 uint32_t *total_runtime);

#endif
