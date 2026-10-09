#include <stdio.h>
#include <math.h>
#include <float.h>
#include "vision_control.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL:%d %s\n", __LINE__, #c); return 1; } } while (0)
int main(void)
{
    VisionControlInput_s input = {.online=1, .mode=1, .now_ms=100, .last_rx_ms=100, .max_age_ms=50};
    CHECK(!VisionControlInputIsUsable(&input));
    VisionControlLimits_s limits = {.configured=1, .yaw_constraint=VISION_YAW_BOUNDED,
        .pitch_min_deg=-20, .pitch_max_deg=30, .pitch_zero_offset_deg=5,
        .yaw_min_deg=-180, .yaw_max_deg=180};
    input.limits = &limits;
    CHECK(VisionControlInputIsUsable(&input));
    limits.pitch_max_deg = -20;
    CHECK(!VisionControlInputIsUsable(&input));
    limits.pitch_max_deg = -21;
    CHECK(!VisionControlInputIsUsable(&input));
    limits.pitch_max_deg = 30;
    limits.configured=0; CHECK(!VisionControlInputIsUsable(&input)); limits.configured=1;
    limits.yaw_constraint=VISION_YAW_UNCONFIGURED;
    CHECK(!VisionControlInputIsUsable(&input));
    limits.yaw_constraint=VISION_YAW_BOUNDED;
    limits.yaw_max_deg=-180; CHECK(!VisionControlInputIsUsable(&input)); limits.yaw_max_deg=180;
    float *bounds[] = {&limits.pitch_min_deg, &limits.pitch_max_deg, &limits.pitch_zero_offset_deg, &limits.yaw_min_deg, &limits.yaw_max_deg};
    for (unsigned i=0;i<5;i++) {
        float saved=*bounds[i];
        *bounds[i]=NAN; CHECK(!VisionControlInputIsUsable(&input));
        *bounds[i]=INFINITY; CHECK(!VisionControlInputIsUsable(&input)); *bounds[i]=saved;
    }
    const float degrees_to_radians=0.017453292519943295f;
    input.pitch=-25 * degrees_to_radians; CHECK(VisionControlInputIsUsable(&input));
    input.pitch-=0.0001f; CHECK(!VisionControlInputIsUsable(&input));
    input.pitch=25 * degrees_to_radians; CHECK(VisionControlInputIsUsable(&input));
    input.pitch+=0.0001f; CHECK(!VisionControlInputIsUsable(&input));
    input.pitch=0;
    input.yaw=-180*degrees_to_radians; CHECK(VisionControlInputIsUsable(&input));
    input.yaw-=0.0001f; CHECK(!VisionControlInputIsUsable(&input));
    input.yaw=180*degrees_to_radians; CHECK(VisionControlInputIsUsable(&input));
    input.yaw+=0.0001f; CHECK(!VisionControlInputIsUsable(&input));
    limits.yaw_constraint=VISION_YAW_CONTINUOUS;
    input.yaw=720*degrees_to_radians; CHECK(VisionControlInputIsUsable(&input));
    limits.yaw_min_deg=NAN; CHECK(!VisionControlInputIsUsable(&input)); limits.yaw_min_deg=-180;
    input.yaw=90*degrees_to_radians; input.pitch=10*degrees_to_radians;
    input.yaw_vel=2*degrees_to_radians; input.yaw_acc=3*degrees_to_radians;
    input.pitch_vel=-2*degrees_to_radians; input.pitch_acc=-3*degrees_to_radians;
    VisionControlGimbalCommand_s command;
    CHECK(VisionControlBuildGimbalCommand(&input,&command));
    CHECK(fabsf(command.yaw-90)<0.0001f && fabsf(command.pitch-15)<0.0001f);
    CHECK(fabsf(command.yaw_vel-2)<0.0001f && fabsf(command.yaw_acc-3)<0.0001f);
    CHECK(fabsf(command.pitch_vel+2)<0.0001f && fabsf(command.pitch_acc+3)<0.0001f);
    float *values[]={&input.yaw,&input.yaw_vel,&input.yaw_acc,&input.pitch,&input.pitch_vel,&input.pitch_acc};
    for (unsigned i=0;i<6;i++) {
        float saved=*values[i];
        *values[i]=NAN; CHECK(!VisionControlInputIsUsable(&input));
        *values[i]=INFINITY; CHECK(!VisionControlInputIsUsable(&input));
        *values[i]=FLT_MAX; CHECK(!VisionControlInputIsUsable(&input)); *values[i]=saved;
    }
    input.mode=2; CHECK(VisionControlInputIsUsable(&input));
    input.mode=0; CHECK(!VisionControlInputIsUsable(&input));
    input.mode=3; CHECK(!VisionControlInputIsUsable(&input)); input.mode=2;
    input.online=0; CHECK(!VisionControlInputIsUsable(&input)); input.online=1;
    input.now_ms=150; CHECK(VisionControlInputIsUsable(&input));
    input.now_ms=151; CHECK(!VisionControlInputIsUsable(&input));
    CHECK(!VisionControlBuildGimbalCommand(&input,&command));
    CHECK(command.yaw==0 && command.pitch==0 && command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    VisionControlShootCommand_s shoot;
    CHECK(!VisionControlBuildShootCommand(&input,&shoot));
    CHECK(shoot.shoot_enabled==0 && shoot.friction_enabled==0 && shoot.load_mode==0 && shoot.shoot_rate==0);
    input.now_ms=0; input.last_rx_ms=0; CHECK(VisionControlInputIsUsable(&input));
    input.last_rx_ms=UINT32_MAX-15; input.now_ms=20; CHECK(VisionControlInputIsUsable(&input));
    input.max_age_ms=51; CHECK(!VisionControlInputIsUsable(&input)); input.max_age_ms=50;
    CHECK(!VisionControlBuildGimbalCommand(NULL,&command));
    CHECK(!VisionControlBuildGimbalCommand(&input,NULL));
    CHECK(!VisionControlInputIsUsable(NULL));
    input.yaw=0; input.pitch=100*degrees_to_radians;
    CHECK(!VisionControlBuildShootCommand(&input,&shoot));
    CHECK(shoot.load_mode==0 && shoot.shoot_enabled==0);
    puts("PASS: missing mechanical configuration rejects vision");
    return 0;
}
