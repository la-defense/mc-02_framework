/**
 * @file vehicle_health_diag.c
 * @brief 将只读健康快照接入USART10的显式诊断请求，不发送日志文本。
 * @note 请求和应答沿用BSPLog已有的D3 6C帧、CRC与复制UART发送队列。
 */
#include "vehicle_health.h"
#include "bsp_log.h"
#include "bsp_log_diag.h"

static uint8_t VehicleHealthBuildReply(uint8_t *output, uint8_t capacity, uint8_t *length)
{
    if ((output == NULL) || (length == NULL))
        return 0u;

    VehicleHealthSnapshot_t snapshot;
    VehicleHealthStatus_t status = VehicleHealthGetSnapshot(&snapshot);
    if (capacity < ((status == VEHICLE_HEALTH_OK)
                        ? (VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE + 2u)
                        : 2u))
        return 0u;

    output[0] = (uint8_t)status;
    output[1] = VEHICLE_HEALTH_DIAGNOSTIC_SCHEMA_VERSION;
    if (status != VEHICLE_HEALTH_OK)
    {
        *length = 2u;
        return 1u;
    }

    uint32_t encoded_length = 0u;
    if (capacity < (VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE + 2u) ||
        VehicleHealthEncodeDiagnostic(&snapshot, output + 2u,
                                      (uint32_t)capacity - 2u, &encoded_length) != VEHICLE_HEALTH_OK ||
        encoded_length != VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE)
        return 0u;

    *length = (uint8_t)(encoded_length + 2u);
    return 1u;
}

VehicleHealthStatus_t VehicleHealthDiagnosticInit(void)
{
    switch (BSPLogRegisterHealthSnapshotHandler(VehicleHealthBuildReply))
    {
    case BSP_LOG_STATUS_OK:
        return VEHICLE_HEALTH_OK;
    case BSP_LOG_STATUS_BAD_ARGUMENT:
        return VEHICLE_HEALTH_ERROR_ARGUMENT;
    case BSP_LOG_STATUS_NOT_INITIALIZED:
        return VEHICLE_HEALTH_ERROR_NOT_INITIALIZED;
    case BSP_LOG_STATUS_BAD_CONTEXT:
        return VEHICLE_HEALTH_ERROR_CONTEXT;
    case BSP_LOG_STATUS_HANDLER_ALREADY_REGISTERED:
        return VEHICLE_HEALTH_ALREADY_INITIALIZED;
    default:
        return VEHICLE_HEALTH_ERROR_CAPTURE;
    }
}
