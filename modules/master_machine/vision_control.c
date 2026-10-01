#include "vision_control.h"

#include <math.h>
#include <stddef.h>

uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input)
{
    const float radians_to_degrees = 57.29577951308232f;
    if (input == NULL || input->online == 0u ||
        (input->mode != 1u && input->mode != 2u) ||
        (uint32_t)(input->now_ms - input->last_rx_ms) > input->max_age_ms)
        return 0u;

    return (uint8_t)(isfinite(input->yaw) && isfinite(input->yaw_vel) &&
                     isfinite(input->yaw_acc) && isfinite(input->pitch) &&
                     isfinite(input->pitch_vel) && isfinite(input->pitch_acc) &&
                     isfinite(input->yaw * radians_to_degrees) &&
                     isfinite(input->yaw_vel * radians_to_degrees) &&
                     isfinite(input->yaw_acc * radians_to_degrees) &&
                     isfinite(input->pitch * radians_to_degrees) &&
                     isfinite(input->pitch_vel * radians_to_degrees) &&
                     isfinite(input->pitch_acc * radians_to_degrees));
}
