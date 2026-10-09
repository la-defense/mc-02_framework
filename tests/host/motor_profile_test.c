#include <stdio.h>
#include "motor_task.h"
#include "profile_io.h"
static unsigned can_pending, uart_pending, usb_pending, dji_outputs, lk_outputs;
void CANProcessRx(void) { if (can_pending) --can_pending; }
void USARTProcessRx(void) { if (uart_pending) --uart_pending; }
void VCPProcessRx(void) { if (usb_pending) --usb_pending; }
void DJIMotorControl(void) { ++dji_outputs; }
void LKMotorControl(void) { ++lk_outputs; }
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)
int main(void)
{
    can_pending = uart_pending = usb_pending = 2;
    MotorControlTask();
    MotorControlTask();
    CHECK(can_pending == 0 && uart_pending == 0 && usb_pending == 0);
#if defined(MC02_PROFILE_BENCH_SAFE)
    CHECK(dji_outputs == 0 && lk_outputs == 0);
#else
    CHECK(dji_outputs == 2 && lk_outputs == 2);
#endif
    puts("PASS: production motor task keeps receive consumers and respects profile isolation");
    return 0;
}
