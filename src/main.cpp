#include <Arduino.h>
#include "main.h"
#include "imu.h"
#include <can.h>
#include "LQR.H"

TaskHandle_t hCAN_Loop; // 开一个线程

static receive_data_t motor_data[2];
float displacement[2] = {0, 0};
int s = 0;
float pitch;
float pitch_velocity;
bool encoder_read = false;
float offsetEcd[2];
float lqr_output1;
float lqr_output2;
float output_limit = 16384;
const float d = 0.106;
const float offsetAng = 0;

// double freq;

// void angleReceive();
void Transmit(int16_t *cmdCurrent);
void Receive();
void CAN_Loop(void *);
// void IMU_readangle();

void setup()
{
   // Setup instance
   Serial.begin(115200);
   IMU_Setup();

   ESP_CAN::CAN can;

   can.CAN_Setup(GPIO_NUM_32, GPIO_NUM_33, b1000k);
   can.CAN_Start();

   // 创建线程
   xTaskCreatePinnedToCore(
       CAN_Loop,
       "can_loop",
       8192, // 开的大点，防止线程数据出现问题
       NULL,
       5, // 优先级问题，与平常所见的不同，数字越大优先级越高
       &hCAN_Loop,
       ARDUINO_EVENT_RUNNING_CORE // ESP的一个模式
   );
}

void CAN_Loop(void *pvParam)
{
   int64_t last_time_cnt;
   int16_t cmdCurrent[2];

   for (;;) // 让线程无限循环下去
   {
      // if (lqr_output1 < 0 && lqr_output2 > 0)
      // {
         cmdCurrent[0] = (int16_t)(lqr_output1) * 176.6 + offsetCurrent[0];
         cmdCurrent[1] = (int16_t)(lqr_output2) * 176.6 + offsetCurrent[1];
      // }
      // else if (lqr_output1 > 0 && lqr_output2 <0)
      // {
      //    cmdCurrent[0] = -((int16_t)(lqr_output1) *0.01 - offsetCurrent[0]);
      //    cmdCurrent[1] = (int16_t)(lqr_output2) *0.01+ offsetCurrent[1];
      // }

      Transmit(cmdCurrent);
   

      vTaskDelay(2);
   }
}

void loop()
{
   pitch = IMU_DataProcess().eul[0] + offsetAng;
   pitch_velocity = -IMU_DataProcess().gyr[0];

   Receive();

   displacement[0] = motor_data[0].turns / 19* pi * d + 
   (motor_data[0].ecd - motor_data[0].last_ecd - offsetEcd[0]) / 19 / 8192 * pi * d;
   // displacement[1] =  (motor_data[0].turns / 19* pi * d + 
   // (motor_data[1].ecd - motor_data[1].last_ecd - offsetEcd[1]) / 19 / 8192 * pi * d); 

   x1 = displacement[0] ;
   x2 = motor_data[0].rotor_spd;
   x5 = motor_data[1].rotor_spd;
   x3 = sin(pitch / 180 * pi);
   x4 = pitch_velocity / 180 * pi;
   u1 = -k1 * x1 - k2 * x2 - k3 * x3 - k4 * x4;
   u2 = -k5 * x1 - k6 * x5 - k7 * x3 - k8 * x4;

   lqr_output1 = m * u1 * L * l / k;
   lqr_output2 = m * u2 * L * l / k;
   // Serial.printf("e_current:%f f_current1:%d  \n f_current2:%d  v1:%d  v2:%d\n", lqr_output * 1000, motor_data[0].torque_current,motor_data[1].torque_current ,motor_data[0].rotor_spd,motor_data[1].rotor_spd);
   vTaskDelay(2);
}

void Transmit(int16_t *cmdCurrent)
{
   // Create variables to store the tx data and rx data
   can_frame_data tx_data;

   // Specify Tx data
   tx_data.identifier = 0x200;
   tx_data.flags = TWAI_MSG_FLAG_NONE;
   tx_data.data_length_code = 8;

   tx_data.data[0] = (uint8_t)(cmdCurrent[0] >> 8);
   tx_data.data[1] = (uint8_t)(cmdCurrent[0]);
   tx_data.data[2] = (uint8_t)(cmdCurrent[1] >> 8);
   tx_data.data[3] = (uint8_t)(cmdCurrent[1]);

   // Transmit
   ESP_CAN::CAN_Transmit(tx_data, 2);
}

void Receive()
{
   can_frame_data rx_message = {0};
   twai_status_info_t twai_stat;

   twai_get_status_info(&twai_stat);

   if (twai_stat.msgs_to_rx != 0)
   {
      ESP_CAN::CAN_Receive(&rx_message, 2);

      if (rx_message.identifier > 0x200 && rx_message.identifier < 0x20D)
      {
         uint8_t id = rx_message.identifier - 0x200;

         motor_data[id - 1].ecd = (uint16_t)(rx_message.data[0] << 8) | rx_message.data[1];
         motor_data[id - 1].rotor_spd = (uint16_t)(rx_message.data[2] << 8) | rx_message.data[3];
         motor_data[id - 1].torque_current = (uint16_t)(rx_message.data[4] << 8) | rx_message.data[5];
         motor_data[id - 1].temperature = rx_message.data[6];

         if (motor_data[0].last_ecd - motor_data[0].ecd - offsetEcd[0]> 0 &&
             motor_data[0].rotor_spd > 0)
         {
            ++motor_data[0].turns;
         }
         else if (motor_data[0].last_ecd - motor_data[0].ecd - offsetEcd[0] < 0 &&
                  motor_data[0].rotor_spd < 0)
         {
            --motor_data[0].turns;
         }

         motor_data[id - 1].last_ecd = motor_data[id - 1].ecd;

         if (!encoder_read)
         {
            offsetEcd[id - 1] = motor_data[id - 1].ecd;
            // offsetEcd[1] = motor_data[1].ecd;
            encoder_read = true;
         }
      }
   }
   // Serial.printf("motor0: %d, cmd0: %d, motor1: %d, cmd1: %d\n", motor_data[0].rotor_spd, (int16_t)-motorSpeedL_pid.get_output(), motor_data[1].rotor_spd, (int16_t)motorSpeedR_pid.get_output());
}