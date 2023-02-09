#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>


#define pi 3.141593
float offsetCurrent[2] = {-270 , 300 };
// float offsetCurrent2=500;
float x1 = 0;
float k1 = -0.3162;
float k5 = -0.3162;
float x2 = 0;
float k2 = -19.3980;
float k6 = -104.4516;
float x3 = 0;
float k3 = 216.6367;
float k7 = 549.1592;
float x4 = 0;
float k4 = 89.3632;
float k8 = 237.4123;
float x5 = 0;
float u1;
float u2;
float k = 0.3;
float m = 4.75;
float L = 0.07;
float l = 0.053;

struct receive_data_t
{
   uint16_t ecd;
   uint16_t last_ecd;
   int64_t turns;
   int16_t rotor_spd;
   int16_t torque_current;
   uint8_t temperature;
};

#endif