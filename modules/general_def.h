#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

// 一些module的通用数值型定义,注意条件macro兼容,一些宏可能在math.h中已经定义过了

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 (PI * 2.0f) // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

#define ANGLE_2_RPM_PER_MIN 0.1666666f // ×60sec/360°
#define RAD_2_RPM_PER_MIN 9.549296596f // ×60sec/2pi

#define MIN_2_SEC 60.0f     // x60sec
#define SEC_2_MIN 0.016666f // x1/60sec

#define MM_2_M 0.001f       // x1/1000mm
#define M_2_MM 1000.0f      // x1000mm

#endif // !GENERAL_DEF_H