#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <can.h>
#include "stdio.h"
#include "string.h"

#define STATE_SOFH 0x00000001
#define STATE_SOFL 0x00000002
#define STATE_DLENH 0x00000003
#define STATE_DLENL 0x00000004
#define STATE_CRC16_H 0x00000005
#define STATE_CRC16_L 0x00000006
#define STATE_DATA 0x000000007

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
#define I2(p) (*((int16_t *)(p)))



union fpf
{
    uint8_t str[4];
    float floatpoint;
};

struct IMU_message
{
    int16_t pitch_raw;
    int16_t roll_raw;
    int16_t yaw_raw;
    float pitch;
    float roll;
    float yaw;
}; //  .h文件不需要声明

struct state_t
{
    struct
    {
        uint8_t SOFH;
        uint8_t SOFL;
        uint16_t DLEN;
        uint16_t CRC16;
    } header_struct;

    uint8_t Data[255];
    uint8_t state;
    uint8_t header_pos;
    bool rx_complete;
};

struct IMU_angle
{
    float pitch;
    float roll;
    float yaw;
};

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} AngleChange;

union u8_to_fp
{
    uint8_t input[4]; //有0~3，4个八位
    float output;
};

/* common type conversion */

typedef struct
{
    uint8_t tag;    /* item tag: 0x91 */
    uint32_t id;    /* user define ID */
    float acc[3];   /* acceleration */
    float gyr[3];   /* angular velocity */
    float mag[3];   /* magnetic field */
    float eul[3];   /* attitude: eular angle */
    float quat[4];  /* attitude: quaternion */
    float pressure; /* air pressure */
    uint32_t timestamp;
} imu_data_t;

static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}
static uint32_t U4(uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}
static int32_t I4(uint8_t *p)
{
    int32_t u;
    memcpy(&u, p, 4);
    return u;
}
static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);//4是长度
    return r;
}

extern void IMU_Setup();
extern void IMU_Receive();
extern imu_data_t IMU_DataProcess();

#endif