#include "imu.h"
#include <Arduino.h>
#include "crc.h"

#define RX_BUFFER_LEN 168u

static u8_to_fp data_converter;
static AngleChange angle;
static state_t state = {
    .state = STATE_SOFH,
    .rx_complete = false}; // state_t状态机

static uint8_t urx_data[RX_BUFFER_LEN];
static uint8_t data_pc = 0;
static imu_data_t i0x91 = {0};

void ClearContents();

void IMU_Setup()
{
    Serial2.begin(115200);
    Serial2.onReceive(IMU_Receive, false);
    Serial2.read(urx_data, RX_BUFFER_LEN);
}

void IMU_Receive()
{
    Serial2.read(urx_data, RX_BUFFER_LEN);

    data_pc = 0;
    state.rx_complete = false;

    while (data_pc < RX_BUFFER_LEN - 1 - 76 && state.rx_complete == false)
    {
        switch (state.state)
        {
        case STATE_SOFH:
            if (urx_data[data_pc] == 0x5A)
            {
                state.header_pos = data_pc;
                state.header_struct.SOFH = urx_data[data_pc];
                state.state = STATE_SOFL; //切换至下一个
            }
            else
            {
                ClearContents(); //清空
                state.state = STATE_SOFH;
            }
            break;
        case STATE_SOFL:
            if (urx_data[data_pc] = 0xA5)
            {
                state.header_struct.SOFL = urx_data[data_pc];
                state.state = STATE_DLENH;
            }
            else
            {
                ClearContents();
                state.state = STATE_SOFH;
            }
            break;
        case STATE_DLENH:
            state.header_struct.DLEN = urx_data[data_pc];
            state.state = STATE_DLENL;
            break;
        case STATE_DLENL:
            state.header_struct.DLEN |= urx_data[data_pc] << 8;
            state.state = STATE_CRC16_H;
            break;
        case STATE_CRC16_H:
            state.header_struct.CRC16 = urx_data[data_pc];
            state.state = STATE_CRC16_L;
            break;
        case STATE_CRC16_L:
            uint16_t crc;
            uint8_t buf[84];

            crc = 0;

            state.header_struct.CRC16 |= urx_data[data_pc] << 8;
            crc16_update(&crc, &urx_data[state.header_pos], 4);
            crc16_update(&crc, &urx_data[state.header_pos] + 6, state.header_struct.DLEN);

            if (crc == state.header_struct.CRC16)
            {
                state.state = STATE_DATA;
            }
            else
            {
                ClearContents();
                state.state = STATE_SOFH;
            }

            break;
        case STATE_DATA:
            memcpy(state.Data, &urx_data[data_pc], sizeof(uint8_t) * state.header_struct.DLEN);
            state.state = STATE_SOFH;
            state.rx_complete = true;
            break;
        default:
            ClearContents();
            state.state = STATE_SOFH;
            break;
        }
        data_pc++;
    }
}

imu_data_t IMU_DataProcess()
{
    
    int offset = 0; /* payload strat at buf[6] */
    i0x91.tag = U1(state.Data + offset + 0);
    i0x91.id = U1(state.Data + offset + 1);
    i0x91.pressure = R4(state.Data + offset + 4);
    i0x91.timestamp = U4(state.Data + offset + 8);
    i0x91.acc[0] = R4(state.Data + offset + 12);
    i0x91.acc[1] = R4(state.Data + offset + 16);
    i0x91.acc[2] = R4(state.Data + offset + 20);
    i0x91.gyr[0] = R4(state.Data + offset + 24);//pitch角加速度
    i0x91.gyr[1] = R4(state.Data + offset + 28);
    i0x91.gyr[2] = R4(state.Data + offset + 32);
    i0x91.mag[0] = R4(state.Data + offset + 36);
    i0x91.mag[1] = R4(state.Data + offset + 40);
    i0x91.mag[2] = R4(state.Data + offset + 44);
    i0x91.eul[0] = R4(state.Data + offset + 48);//pitch角度
    i0x91.eul[1] = R4(state.Data + offset + 52);
    i0x91.eul[2] = R4(state.Data + offset + 56);
    i0x91.quat[0] = R4(state.Data + offset + 60);
    i0x91.quat[1] = R4(state.Data + offset + 64);
    i0x91.quat[2] = R4(state.Data + offset + 68);
    i0x91.quat[3] = R4(state.Data + offset + 72);

    // Serial.printf("header1: %x\t", state.header_struct.SOFH);
    // Serial.printf("header2: %x\t", state.header_struct.SOFL);
    // Serial.printf("datalen: %x\t", state.header_struct.DLEN);
    // Serial.printf("crc: %x\t", state.header_struct.CRC16);

    // Serial.printf("p:%.2f r:%.2f y:%.2f\n", i0x91.eul[0], i0x91.eul[1], i0x91.eul[2]);

    return i0x91;
}

void ClearContents()
{
    state.header_struct.SOFH = 0x00;
    state.header_struct.SOFL = 0x00;
    state.header_struct.DLEN = 0x00;
    state.header_struct.CRC16 = 0x00;

    memset(state.Data, 0x00, sizeof(state.Data));

    state.rx_complete = false;
    state.state = STATE_SOFH;
}