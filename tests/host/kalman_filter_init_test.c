#include "kalman_filter.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static long allocation_countdown = -1;
static size_t live_allocations;
static int matrix_init_countdown = -1;

void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        allocation_countdown--;
    void *pointer = __real_malloc(size);
    if (pointer != NULL)
        live_allocations++;
    return pointer;
}
void *__wrap_calloc(size_t count, size_t size)
{
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        allocation_countdown--;
    void *pointer = __real_calloc(count, size);
    if (pointer != NULL)
        live_allocations++;
    return pointer;
}
void __wrap_free(void *pointer)
{
    if (pointer != NULL)
    {
        assert(live_allocations > 0u);
        live_allocations--;
    }
    __real_free(pointer);
}

void arm_mat_init_f32(arm_matrix_instance_f32 *matrix, uint16_t rows, uint16_t cols, float *data)
{
    if (matrix_init_countdown == 0)
        return;
    if (matrix_init_countdown > 0)
        matrix_init_countdown--;
    matrix->numRows = rows;
    matrix->numCols = cols;
    matrix->pData = data;
}
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *c)
{ (void)a; (void)b; (void)c; return ARM_MATH_SUCCESS; }
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *c)
{ (void)a; (void)b; (void)c; return ARM_MATH_SUCCESS; }
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *c)
{ (void)a; (void)b; (void)c; return ARM_MATH_SUCCESS; }
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b)
{ (void)a; (void)b; return ARM_MATH_SUCCESS; }
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b)
{ (void)a; (void)b; return ARM_MATH_SUCCESS; }

int main(void)
{
    KalmanFilter_t filter;
    memset(&filter, 0xA5, sizeof(filter));
    KalmanFilter_t untouched = filter;
    assert(!Kalman_Filter_Init(NULL, 6u, 0u, 3u));
    assert(!Kalman_Filter_Init(&filter, 0u, 0u, 3u));
    assert(memcmp(&filter, &untouched, sizeof(filter)) == 0);

    for (long fail_after = 0; fail_after < 24; ++fail_after)
    {
        allocation_countdown = fail_after;
        size_t before = live_allocations;
        if (Kalman_Filter_Init(&filter, 6u, 0u, 3u))
        {
            fprintf(stderr, "allocation fault injection did not fail at allocation %ld\n", fail_after);
            return 1;
        }
        allocation_countdown = -1;
        assert(live_allocations == before);
        assert(memcmp(&filter, &untouched, sizeof(filter)) == 0);
    }

    for (int fail_matrix = 0; fail_matrix < 17; ++fail_matrix)
    {
        matrix_init_countdown = fail_matrix;
        size_t before = live_allocations;
        assert(!Kalman_Filter_Init(&filter, 6u, 0u, 3u));
        matrix_init_countdown = -1;
        assert(live_allocations == before);
        assert(memcmp(&filter, &untouched, sizeof(filter)) == 0);
    }

    assert(Kalman_Filter_Init(&filter, 6u, 0u, 3u));
    assert(filter.uSize == 0u && filter.ControlVector == NULL && filter.u_data == NULL);
    assert(live_allocations > 0u);
    Kalman_Filter_Release(&filter);
    assert(live_allocations == 0u);
    return 0;
}
