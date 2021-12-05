//#include <MPU6050_tockn.h>
#include <MPU6050_light.h>
#include <GpioExpander.h>
#include <i2cioCommands.h>
#include <TimeLib.h>
#include <stdio.h>
#include <string.h>
#include <Octoliner.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include "I2C_Anything.h"
#include <Servo.h>
#include <Ultrasonic.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
MPU6050 mpu(Wire);
Servo myservo_1;
Servo myservo_2;
Servo myservo_3;
Servo myservo_4;
Adafruit_NeoPixel *pixels_left;
Adafruit_NeoPixel *pixels_right;
Octoliner octoliner(42);
Ultrasonic d_forward_right(35, 37);
Ultrasonic d_backward_right (31, 33);
Ultrasonic d_forward_left (43, 45);
Ultrasonic d_backward_left (47, 49);
Ultrasonic d_middle_right(A0, A3);
Ultrasonic d_middle_left (A2, A1);
Ultrasonic d_forward_up (51, 53);
Ultrasonic d_forward_down (39, 41);
int id1, id2, id3, id4, id5;
int numPixels   = 8;
int pixelFormat = NEO_GRB + NEO_KHZ800;
int pin_led_left = 6;
int pin_led_right = 7;
int Speed = 125;
char btCommand = 'S';
int speed_11;
int speed_22;
int speed_33;
int speed_44;
int knob_r_edge;
int knob_l_edge;
int dataSensors[8];
float line_data;
int angleX, angleY, angleZ;
int basket_down_angle = 161;
int basket_up_angle = 140;
int entry_open_angle = 130;
int entry_middle_angle = 80;
int entry_close_angle = 40;
int hvat_up_angle = 135;
int hvat_down_angle = 25;
int hvat_open_angle = 125;
int hvat_close_angle = 175;
int dist_right_middle, dist_left_middle;
int s1_black = 996;
int s2_black = 994;
int s3_black = 984;
int s4_black = 993;
int s5_black = 985;
int s6_black = 978;
int s7_black = 987;
int s8_black = 990;

int s1_white = 908;
int s2_white = 865;
int s3_white = 840;
int s4_white = 805;
int s5_white = 772;
int s6_white = 804;
int s7_white = 826;
int s8_white = 867;

int s1_gray = 783;
int s2_gray = 746;
int s3_gray = 186;
int s4_gray = 87;
int s5_gray = 58;
int s6_gray = 601;
int s7_gray = 495;
int s8_gray = 635;

int limit_1_g = (s1_white + s1_gray) / 2;
int limit_2_g = (s2_white + s2_gray) / 2;
int limit_3_g = (s3_white + s3_gray) / 2;
int limit_4_g = (s4_white + s4_gray) / 2;
int limit_5_g = (s5_white + s5_gray) / 2;
int limit_6_g = (s6_white + s6_gray) / 2;
int limit_7_g = (s7_white + s7_gray) / 2;
int limit_8_g = (s8_white + s8_gray) / 2;
float angle_x;
float angle_y;
int limit_1 = (s1_black + s1_white) / 2;
int limit_2 = (s2_black + s2_white) / 2;
int limit_3 = (s3_black + s3_white) / 2;
int limit_4 = (s4_black + s4_white) / 2;
int limit_5 = (s5_black + s5_white) / 2;
int limit_6 = (s6_black + s6_white) / 2;
int limit_7 = (s7_black + s7_white) / 2;
int limit_8 = (s8_black + s8_white) / 2;
#define knob_plat_ru A4
#define knob_plat_lu 29
#define knob_plat_rd 14
#define knob_plat_ld 17
int angle_zero;
int knob_p_ru, knob_p_rd, knob_p_lu, knob_p_ld;

int knob_plat_mode_1 = 15;
int knob_plat_mode_2 = 16;

int knob_pm_1, knob_pm_2, mode;

int id_per_future, id_per_last = 0;
int sbor_value1, sbor_value2, sbor_value3, sbor_value4;
int spv1, spv2, spv3, spv4;
int enclu = 3, encld = 19, encru = 2, encrd = 18;
int encLU = 1, encLD = 5, encRU = 0, encRD = 4;
int enc_up_L = 0, enc_down_L = 0, enc_up_R = 0, enc_down_R = 0;
int dist_right_forward;
int dist_right_back;
int dist_left_forward;
int dist_left_back;
int dist_middle_right;
int dist_middle_left;
int dist_forward_up;
int dist_forward_down;
#define enA_Left 5
#define in1_Left A15//zad
#define in2_Left A14
#define in3_Left A12
#define in4_Left A13
#define enB_Left 8
#define enA_Right 10
#define in1_Right A10//zad
#define in2_Right A11
#define in3_Right A9
#define in4_Right A8
#define enB_Right 9
unsigned long timer = millis();
float tik_in_ob = 1725;
float diam = 8.5;
float tik_in_5_sec_max = 13718.4;
float tik_in_1_sec_max = tik_in_5_sec_max / 5;
float l_col = 3.14 * diam;
float max_speed_cm_sec = (tik_in_1_sec_max * (l_col / tik_in_ob));
float cm_in_1_tik = (l_col / tik_in_ob);
float k_pwm_cm = 255 / max_speed_cm_sec;
int time_delay = 10;
int stp = 1;
int barrier_sensor_pin = A5;
int barrier;
int Map[6][8] = {{0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};
int dst_long = 95;
int dst_short = 60;
int limit_wall = (dst_long + dst_short) / 2;
float kp1 =   6;
float ki1 = 0;
float kd1 = 0.01;
float kp2 = 6;
float ki2 = 0;
float kd2 = 0.01;
float kp3 = 6;
float ki3 = 0;
float kd3 = 0.01;
float kp4 = 6;
float ki4 = 0;
float kd4 = 0.01;
int speed_PID_LU, speed_PID_LD, speed_PID_RU, speed_PID_RD;
float real_speed_LU;
float real_speed_LD;
float real_speed_RU;
float real_speed_RD;
int computePID_LU(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut)
{
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}
int computePID_LD(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut)
{
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}
int computePID_RU(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut)
{
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}
int computePID_RD(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut)
{
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

void forward () // ВПЕРЕД
{
    speed_up_left(Speed);
    speed_down_left(Speed);
    speed_up_right(Speed);
    speed_down_right(Speed);
}
void left () // ПОВОРОТ ВПРАВО (одна сторона) void right (int b)
{
speed_up_left(-Speed);
    speed_down_left(-Speed);
    speed_up_right(Speed);
    speed_down_right(Speed);
}
void right  () // ПОВОРОТ ВЛЕВО (одна сторона) void left (int c)
{
    speed_up_left(Speed);
    speed_down_left(Speed);
    speed_up_right(-Speed);
    speed_down_right(-Speed);
}
//void turnR () // РАЗВОРОТ ВПРАВО (два стороны)
//{
//digitalWrite (MotorRightBack, HIGH);
//digitalWrite (MotorRightForward, LOW);
//digitalWrite (MotorLeftBack, LOW);
//digitalWrite (MotorLeftForward, HIGH);
//}
//void turnL () // РАЗВОРОТ ВЛЕВО (два стороны)
//{
//digitalWrite (MotorRightBack, LOW);
//digitalWrite (MotorRightForward, HIGH);
//digitalWrite (MotorLeftBack, HIGH);
//digitalWrite (MotorLeftForward, LOW);
//}
void stopp () // СТОП
{
speed_up_left(0);
    speed_down_left(0);
    speed_up_right(0);
    speed_down_right(0);
}
void back () // НАЗАД
{
speed_up_left(-Speed);
    speed_down_left(-Speed);
    speed_up_right(-Speed);
    speed_down_right(-Speed);
}
void setup()
{
  delay(500);
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  //mpu6050.calcGyroOffsets(true);
  octoliner.begin();
  delay(1000);
  octoliner.setSensitivity(185);
  pixels_left = new Adafruit_NeoPixel(numPixels, pin_led_left, pixelFormat);
  pixels_right = new Adafruit_NeoPixel(numPixels, pin_led_right, pixelFormat);
  pixels_left->begin();
  pixels_right->begin();
  myservo_1.attach(11);
  myservo_2.attach(4);
  myservo_3.attach(13);
  myservo_4.attach(12);
  myservo_2.write(hvat_close_angle);
  myservo_1.write(hvat_up_angle);
  myservo_3.write(basket_down_angle);
  myservo_4.write(entry_close_angle);
  pinMode(A7, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  pinMode(knob_plat_ru, INPUT_PULLUP);
  pinMode(knob_plat_rd, INPUT_PULLUP);
  pinMode(knob_plat_lu, INPUT_PULLUP);
  pinMode(knob_plat_ld, INPUT_PULLUP);

  pinMode(knob_plat_mode_1, INPUT_PULLUP);
  pinMode(knob_plat_mode_2, INPUT_PULLUP);
  pinMode(in1_Right, OUTPUT);
  pinMode(in2_Right, OUTPUT);
  pinMode(in3_Right, OUTPUT);
  pinMode(in4_Right, OUTPUT);
  pinMode(enA_Right, OUTPUT);
  pinMode(enB_Right, OUTPUT);
  pinMode(in1_Left, OUTPUT);
  pinMode(in2_Left, OUTPUT);
  pinMode(in3_Left, OUTPUT);
  pinMode(in4_Left, OUTPUT);
  pinMode(enA_Left, OUTPUT);
  pinMode(enB_Left, OUTPUT);
  //read_MPU6050();
  //delay(4000);
  delay(100);
  //angle_zero = angleZ;
  led_stop();
  stop_motor();
  setup_stop();
  //     set_color_left(255, 255, 255);
  //    set_color_right(255, 255, 255);
}
void loop()
{
  get_data();
  while(id4 != 1)
  {
    go_line_octoliner(200);
    get_data();
  }

  while(true)
  {
    stop_motor();
  }
  //fake_line_v2();
  //fake_line_v1();
  //serial_sensors();
  ///////////////////////////////////////////V3
//  go_to_dist(15);
//  stop_motor();
//  delay(100);
//rotation_giro(-45);
//go_to_dist(2);
//rotation_giro(90);
//go_to_dist(4);
//rotation_giro(-90);
//go_to_dist(4);
//rotation_giro(90);
//go_to_dist(3);
//rotation_giro(-90);
//go_to_dist(4);
//rotation_giro(90);
//go_to_dist(3);
//rotation_giro(-90);
//go_to_dist(4);
//rotation_giro(37);
//stop_motor();
//go_to_dist(5);
//////////////////////////////////////////////

  
}
void fake_line_v2()
{
  read_mode();
  if ( mode == 1)////////////////////////////////////////////////////////////////////   mod===================================11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
  {
    go_to_dist(70);
    
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
 
    rotation_giro(170);
    long int tttimer = millis();
    long int ttimer = millis();
    while (tttimer - ttimer < 7000)
    {
      tttimer = millis();
      go_line_octoliner(200);
    }
    line_sensor();
    while (dataSensors[1] < limit_1)
    {
      go_line_octoliner(200);
    }
    rotation_giro(60);
    go_to_dist(2);
    ///////////////////////////////////////////////////////////////////////////////
    int cfrt = 1;
    while (cfrt == 1)
    {
      if (dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
      {
        cfrt = 2;
      }
      go_line_octoliner(200);
    }
    line_sensor();
    go_to_dist(30);
    //////////////////////////////////////////////////////////////////////////////////
    rotation_giro(-45);
    go_to_dist(10);
    rotation_giro(-45);
    go_to_dist(3);
    rotation_giro(-45);
    go_to_dist(3);
    go_to_dist(-3);
    rotation_giro(37);
    go_to_dist(10);
    line_sensor();
    while (dataSensors[8] < limit_8 && dataSensors[7] < limit_7)
    {
      line_sensor();
      speed_up_left(170);
      speed_down_left(170);
      speed_up_right(170);
      speed_down_right(170);
    }
    rotation_giro(-45);
    go_to_dist(2);
    rotation_giro(60);
    line_sensor();
    int cfrrt = 1;
    while (cfrrt == 1)
    {
      if (dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
      {
        cfrrt = 2;
      }
      line_sensor();
      go_line_octoliner(200);
    }
    go_to_dist(54);
    rotation_giro(-45);
    go_to_dist(19);
    rotation_giro(-45);
    go_to_dist(20);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    rotation_giro(170);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(1);
    rotation_giro(85);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    rotation_giro(170);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(2);
    rotation_giro(85);
    go_to_dist(2);
    //delay(50);
    read_light();
    while (barrier == 1)
    {
      go_line_octoliner(200);
      read_light();
    }
    go_angle_giro(0);
    go_to_dist(-5);
    line_rovno();
    rotation_giro(60);
    go_to_dist(18);
    rotation_giro(-60);
    go_to_dist(30);
    rotation_giro(-60);
    go_to_dist(27);
    rotation_giro(50);



    get_data();
    while (id4 == 0)
    {
      speed_up_left(170);
      speed_down_left(170);
      speed_up_right(170);
      speed_down_right(170);
       get_data();
    }
    while (true)
      stop_motor();
  }
  else if (mode == 2)////////////////////////////////////////////////////////////////////// mod ================================ 22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
  {
//   int cfrtt = 1;
//    while (cfrtt == 1)
//    {
//      if (dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
//      {
//        cfrtt = 2;
//      }
//      go_line_octoliner(200);
//    }
//    line_sensor();
    go_to_dist(48);
    //////////////////////////////////////////////////////////////////////////////////
    rotation_giro(-45);
    go_to_dist(10);
    rotation_giro(-45);
    go_to_dist(3);
    rotation_giro(-45);
    go_to_dist(3);
    go_to_dist(-3);
    rotation_giro(45);
    go_to_dist(5);
    line_sensor();
    while (dataSensors[8] < limit_8 && dataSensors[7] < limit_7)
    {
      line_sensor();
      speed_up_left(170);
      speed_down_left(170);
      speed_up_right(170);
      speed_down_right(170);
    }
    rotation_giro(-45);
    go_to_dist(2);
    rotation_giro(60);
    line_sensor();
    int cfrrt = 1;
    while (cfrrt == 1)
    {
      if (dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
      {
        cfrrt = 2;
      }
      line_sensor();
      go_line_octoliner(200);
    }
    go_to_dist(54);
    rotation_giro(-45);
    go_to_dist(19);
    rotation_giro(-45);
    go_to_dist(20);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    rotation_giro(170);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(1);
    rotation_giro(85);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    rotation_giro(170);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(2);
    rotation_giro(85);
    go_to_dist(2);
    //delay(50);
    read_light();
    while (barrier == 1)
    {
      go_line_octoliner(200);
      read_light();
    }
    go_angle_giro(0);
    go_to_dist(-5);
    line_rovno();
    rotation_giro(60);
    go_to_dist(18);
    rotation_giro(-60);
    go_to_dist(30);
    rotation_giro(-60);
    go_to_dist(27);
    rotation_giro(50);



    get_data();
    while (id4 == 0)
    {
      speed_up_left(170);
      speed_down_left(170);
      speed_up_right(170);
      speed_down_right(170);
       get_data();
    }
    while (true)
      stop_motor();
  }
  else if (mode == 3)////////////////////////////////////////////////////////////////////// mod ============================ 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
  {
    /////////////////////////////////////////////////////////////
      go_to_dist(15);
  stop_motor();
 // delay(100);
rotation_giro(-45);
go_to_dist(2);
rotation_giro(90);
go_to_dist(4);
rotation_giro(-90);
go_to_dist(4);
rotation_giro(90);
go_to_dist(3);
rotation_giro(-90);
go_to_dist(4);
rotation_giro(90);
go_to_dist(3);
rotation_giro(-90);
go_to_dist(4);
rotation_giro(38);
stop_motor();
go_to_dist(8);
//////////////////////////////////////////////////////////////////////////////////
     go_to_dist(54);
    rotation_giro(-45);
    go_to_dist(2e1);
    rotation_giro(-45);
    go_to_dist(20);
//    line_sensor();
//    while (dataSensors[8] < limit_8)
//    {
//      go_line_octoliner(200);
//    }
//    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    rotation_giro(170);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(1);
    rotation_giro(85);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    rotation_giro(170);
    go_to_dist(3);
    line_sensor();
    while (dataSensors[1] < limit_1 || dataSensors[8] < limit_8)
    {
      go_line_octoliner(200);
      line_sensor();
    }
    go_to_dist(2);
    rotation_giro(85);
    go_to_dist(2);
   // delay(50);
    read_light();
    while (barrier == 1)
    {
      go_line_octoliner(200);
      read_light();
    }
    go_angle_giro(0);
    go_to_dist(-5);
    line_rovno();
    rotation_giro(60);
    go_to_dist(18);
    rotation_giro(-60);
    go_to_dist(30);
    rotation_giro(-60);
    go_to_dist(27);
    rotation_giro(50);



    get_data();
    while (id4 == 0)
    {
      speed_up_left(170);
      speed_down_left(170);
      speed_up_right(170);
      speed_down_right(170);
       get_data();
    }



    while (true)
      stop_motor();
  }
}
void go_angle_giro(int ang)
{

//  read_MPU6050();//вперед
//  //angleZ = angleZ/2;
//  //ang = ang * 2;
//
//  ////////////////////////////////////////////////////////////////////
////  if (angleZ > 700)
////  {
////    angleZ = angleZ - 700;
////  }
////  else if (angleZ > 1400)
////  {
////    angleZ = angleZ - 1400;
////  }
////  else if (angleZ > 2100)
////  {
////    angleZ = angleZ - 2100;
////  }
////  else if (angleZ > 2800)
////  {
////    angleZ = angleZ - 2800;
////  }
////
////  if (angleZ < 0)
////  {
////    if (angleZ > -700)
////      angleZ = angleZ + 700;
////    else if ( angleZ > -1400)
////      angleZ = angleZ + 1400;
////    else if (angleZ > -2100)
////      angleZ = angleZ + 2100;
////    else if (angleZ > -2800)
////      angleZ = angleZ + 2800;
////  }
//
////////////////////////////////////////////////////////////////////////
//  if ( ang != 0)
//  {
//    while (angleZ < ang - 2 || angleZ > ang + 2)
//    {
//      read_MPU6050();
//      /////////////////////////////////////////////////////////////
//          if (angleZ > 360)
//    {
//      angleZ = angleZ - 360;
//    }
//    else if (angleZ > 720)
//    {
//      angleZ = angleZ - 720;
//    }
//    else if (angleZ > 1080)
//    {
//      angleZ = angleZ - 1080;
//    }
//    else if (angleZ > 1440)
//    {
//      angleZ = angleZ - 1440;
//    }
//  
//  if (angleZ < 0)
//  {
//    if (angleZ > -360)
//    angleZ = angleZ+360;
//    else if ( angleZ > -720)
//      angleZ = angleZ+720;
//    else if (angleZ > -1080)
//    angleZ = angleZ + 1080;
//    else if (angleZ > -1440)
//    angleZ = angleZ + 1440;
//  }
//      ////////////////////////////////////////////////////////////////
//      if (angleZ > ang)
//      {
//        speed_up_left(170);
//        speed_down_left(170);
//        speed_up_right(-170);
//        speed_down_right(-170);
//      }
//      else if (angleZ < ang)
//      {
//        speed_up_left(-170);
//        speed_down_left(-170);
//        speed_up_right(170);
//        speed_down_right(170);
//      }
//    }
//    stop_motor();
//  }
//  else if (ang == 0)
//  {
//    while ((angleZ < 358 && angleZ > 180) || (angleZ > ang + 2 && angleZ <= 180))
//    {
//      read_MPU6050();
//      ////////////////////////////////////////////////////////////////////////////
//          if (angleZ > 360)
//    {
//      angleZ = angleZ - 360;
//    }
//    else if (angleZ > 720)
//    {
//      angleZ = angleZ - 720;
//    }
//    else if (angleZ > 1080)
//    {
//      angleZ = angleZ - 1080;
//    }
//    else if (angleZ > 1440)
//    {
//      angleZ = angleZ - 1440;
//    }
//  
//  if (angleZ < 0)
//  {
//    if (angleZ > -360)
//    angleZ = angleZ+360;
//    else if ( angleZ > -720)
//      angleZ = angleZ+720;
//    else if (angleZ > -1080)
//    angleZ = angleZ + 1080;
//    else if (angleZ > -1440)
//    angleZ = angleZ + 1440;
//  }
//      ////////////////////////////////////////////////////////////////////////////////
//      if (angleZ > ang + 2 && angleZ < 150)
//      {
//        speed_up_left(170);
//        speed_down_left(170);
//        speed_up_right(-170);
//        speed_down_right(-170);
//      }
//      else if (angleZ < 358 && angleZ > 150)
//      {
//        speed_up_left(-170);
//        speed_down_left(-170);
//        speed_up_right(170);
//        speed_down_right(170);
//      }
//    }
//    stop_motor();
//  }
//  // delay(500);
//  // }
//  //  else if ( ang == 90)//влево
//  //  {
//  //
//  //  }
//  //  else if ( ang == 180)//назад
//  //  {
//  //
//  //  }
//  //  else if ( ang == 270)//впрево
//  //  {
//  //
//  //  }
}
void setup_stop()
{
  get_data();
  while (id1 == 255 || id2 == -1)
  {
    get_data();
  }
}
void get_data()
{
  Wire.requestFrom(8, 10);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    id1 = Wire.read ();
    id1 <<= 10;
    id1 |= Wire.read ();

    id2 = Wire.read ();
    id2 <<= 10;
    id2 |= Wire.read ();

    id3 = Wire.read ();
    id3 <<= 10;
    id3 |= Wire.read ();

    id4 = Wire.read ();
    id4 <<= 10;
    id4 |= Wire.read ();

    id5 = Wire.read ();
    id5 <<= 10;
    id5 |= Wire.read ();
    //
    id_per_future = id5;

    if (id3 < 0)
    {
      id3 = id3 + 769;
    }
//    Serial.print(id1); Serial.print("   ");
//    Serial.print(id2); Serial.print("   ");
//    Serial.print(id3); Serial.print("   ");
//    Serial.print(id4); Serial.print("   ");
//    Serial.println(id5);
  }
}
void fake_line_v1()
{
  go_to_dist(27);
  rotation_giro(-85);
  stop_motor();
  //delay(50);
  //line_rovno();
  go_to_dist(90);
  rotation_giro(175);
  stop_motor();
  //delay(50);
  //line_rovno();
  go_to_dist(55);
  rotation_giro(85);
  stop_motor();
 // delay(50);
  go_to_dist(17);
  rotation_giro(45);
  go_to_dist(12);
  rotation_giro(45);
  go_to_dist(5);
  line_sensor();
  while (dataSensors[3] > limit_3_g && dataSensors[4] > limit_4_g && dataSensors[5] > limit_5_g && dataSensors[6] > limit_6_g)
  {
    go_line_octoliner(200);
  }
  fake_zone1();
 // rotation_giro(25);
  read_knob_plat();
  while (knob_r_edge == 1)
  {
    read_knob_plat();
    go_line_octoliner(200);
  }
  //  speed_up_left(-170);
  //  speed_down_left(-170);
  //  //speed_up_right(170);
  //  //speed_down_right(170);
  //  delay(500);
  go_to_dist(-1);

  speed_up_left(-170);
  speed_down_left(-170);
  speed_up_right(200);
  speed_down_right(200);
  delay(300);
  speed_up_left(0);
  speed_down_left(0);
  speed_up_right(200);
  speed_down_right(200);
  delay(450);
  go_to_dist(4);
  get_data();
  while (id4 == 0)
  {
    get_data();
    go_forward_right();
  }
  go_to_dist(1);
  while (true)
  {
    stop_motor();
  }
}
void objezd()
{
    //go_angle_giro(0);
    go_to_dist(-5);
    line_rovno();
    rotation_giro(60);
    go_to_dist(18);
    rotation_giro(-60);
    go_to_dist(30);
    rotation_giro(-60);
    go_to_dist(24);
    rotation_giro(60);


    get_data();
    while (id4 == 0)
    {
      speed_up_left(170);
      speed_down_left(170);
      speed_up_right(170);
      speed_down_right(170);
       get_data();
    }
}
void perecrest()
{
  get_data();
  if (id_per_future != id_per_last && id1 != 0)
  {
    stop_motor();
    if (id1 == 1)
    {
      go_to_dist(15);
      rotation_giro(85);
      go_to_dist(5);
    }
    else if (id1 == 2)
    {
      go_to_dist(15);
      rotation_giro(-85);
      go_to_dist(5);
    }
    else if (id1 == 3)
    {
      go_to_dist(15);
      rotation_giro(170);
      go_to_dist(5);
    }
    id_per_future = id_per_last;
  }
}
void event_from_camera1()
{
  get_data();
  //stop_motor();
  //delay(100);
  //get_data();
  if (id2 != 0)
  {
    stop_motor();
    //подбор объекта
    if (id3 > 0 )
    {
      rotation_giro_special(-(id3 - 10));
      //        rotation_giro(-id3);
      hvat_down();
      go_to_dist(id2 - 5);
      hvat_up();
      // подбор завершен
      shaking();
      go_to_dist(-(id2 - 5));
      rotation_giro(id3 - 10);
      stop_motor();
    }
    else if (id3 < 0 )
    {
      // id3 = id3 + 769;
      rotation_giro_special(id3 - 10);
      hvat_down();
      go_to_dist(id2 - 5);
      hvat_up();
      shaking();
      go_to_dist(-(id2 - 5));
      rotation_giro(-(id3 - 10));
      stop_motor();
    }
    else
    {
      hvat_down();
      go_to_dist(id2 - 5);
      hvat_up();
      shaking();
      go_to_dist(-(id2 - 5));
      stop_motor();
    }
  }
  else if (id_per_future != id_per_last && id1 != 0)
  {
    //get_data();
    //поворот
    //0-прямо
    //1-влево
    //2-вправо
    //3-назад
    stop_motor();
    line_rovno();
    get_data();
    //go_to_dist(15);
    if (id1 == 1)
    {
      go_to_dist(15);
      rotation_giro(85);
      go_to_dist(5);
    }
    else if (id1 == 2)
    {
      go_to_dist(15);
      rotation_giro(-85);
      go_to_dist(5);
    }
    else if (id1 == 3)
    {
      go_to_dist(15);
      rotation_giro(170);
      go_to_dist(5);
    }
    id_per_last = id_per_future;
    stop_motor();
  }
  //  else if (id_per_future != id_per_last && id1 == 0)
  //  {
  //    //поворот
  //    //0-прямо
  //    stop_motor();
  //    line_rovno();
  //    get_data();
  //    go_to_dist(5);
  //    id_per_last = id_per_future;
  //    stop_motor();
  //  }
  else if (id4 != 0)
  {
    //остановка
    stop_motor();
    set_color_left(255, 0, 0);
    delay(10000);
    led_stop();
    delay(500);
  }
  id1 = 0;
  id2 = 0;
  id3 = 0;
}
void povorot_on_angle_camera(int angle_camera)
{

  if (angle_camera < 0)
  {
    int angle_camera_real = angle_camera + 50;
    //первый вариант поворот влево
    rotation_giro(angle_camera_real);
    line_sensor();
    if (dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
    {
      while (dataSensors[5] < limit_5 && dataSensors[4] < limit_4)
      {
        line_sensor();
        speed_up_left(-170);
        speed_down_left(-170);
        speed_up_right(170);
        speed_down_right(170);
      }
      stop_motor();
    }
  }
  else if (angle_camera > 0)
  {
    int angle_camera_real = angle_camera - 50;
    //первый вариант поворот вправо
    rotation_giro(angle_camera_real);
    line_sensor();
    if (dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3)
    {
      while (dataSensors[5] < limit_5 && dataSensors[4] < limit_4)
      {
        line_sensor();
        speed_up_left(170);
        speed_down_left(170);
        speed_up_right(-170);
        speed_down_right(-170);
      }
      stop_motor();
    }
  }
}
void line_rovno()
{
  line_sensor();
  while ( dataSensors[4] < limit_4 || dataSensors[5] < limit_5) // |0|0|0| | |0|0|0|
  {
    line_sensor();
    if (dataSensors[1] > limit_1 || dataSensors[2] > limit_2 || dataSensors[3] > limit_3)//левее
    {
      speed_up_left(-160);
      speed_down_left(-160);
      speed_up_right(160);
      speed_down_right(160);
    }
    else if (dataSensors[6] > limit_6 && dataSensors[7] > limit_7 || dataSensors[8] > limit_8) //правее
    {
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(-160);
      speed_down_right(-160);
    }
    if (dataSensors[1] > limit_1 || dataSensors[2] > limit_2 || dataSensors[3] > limit_3)//левее
    {
      speed_up_left(-160);
      speed_down_left(-160);
      speed_up_right(160);
      speed_down_right(160);
    }
    else if (dataSensors[6] > limit_6 && dataSensors[7] > limit_7 || dataSensors[8] > limit_8) //правее
    {
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(-160);
      speed_down_right(-160);
    }
  }
  stop_motor();
}

void rotation_giro_special(int angle_01)
{
  read_MPU6050();
  int last_angle = angleZ;
  if (angle_01 > 0)
  {
    read_MPU6050();
    while (angleZ - last_angle < angle_01)
    {
      read_MPU6050();
      speed_up_left(-150);
      speed_down_left(-150);
      speed_up_right(150);
      speed_down_right(150);
    }
  }
  else if (angle_01 < 0)
  {
    read_MPU6050();
    while (angleZ - last_angle > angle_01)
    {
      read_MPU6050();
      speed_up_left(150);
      speed_down_left(150);
      speed_up_right(-150);
      speed_down_right(-150);
    }
  }
  stop_motor();
}
void read_MPU6050()
{
  mpu.update();
  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();
         if (angleZ > 360)
    {
      angleZ = angleZ - 360;
    }
    else if (angleZ > 720)
    {
      angleZ = angleZ - 720;
    }
    else if (angleZ > 1080)
    {
      angleZ = angleZ - 1080;
    }
    else if (angleZ > 1440)
    {
      angleZ = angleZ - 1440;
    }
  
  if (angleZ < 0)
  {
    if (angleZ > -360)
    angleZ = angleZ+360;
    else if ( angleZ > -720)
      angleZ = angleZ+720;
    else if (angleZ > -1080)
    angleZ = angleZ + 1080;
    else if (angleZ > -1440)
    angleZ = angleZ + 1440;
  }
//  Serial.print (angleX);
//  Serial.print(" ");
//  Serial.print (angleY);
//  Serial.print(" ");
//  Serial.print (angleZ);
//  Serial.println(" ");
}
void read_MPU6050_old()
{
//  mpu6050.update();
//  angleX = mpu6050.getAngleX();
//  angleY = mpu6050.getAngleY();
//  angleZ = mpu6050.getAngleZ();
//
//  Serial.print (angleX);
//  Serial.print(" ");
//  Serial.print (angleY);
//  Serial.print(" ");
//  Serial.print (angleZ);
//  Serial.println(" ");

}
void go_forward_right()
{
  read_distance();
  if (dist_right_forward > dist_right_back)
  {
    speed_up_left(180);
    speed_down_left(180);
    speed_up_right(0);
    speed_down_right(0);
  }
  else if (dist_right_forward < dist_right_back)
  {
    speed_up_left(0);
    speed_down_left(0);
    speed_up_right(180);
    speed_down_right(180);
  }
  else if (dist_right_forward == dist_right_back)
  {
    speed_up_left(180);
    speed_down_left(180);
    speed_up_right(180);
    speed_down_right(180);
  }

}
void go_forward_left()
{
  read_distance();
  if (dist_left_forward > dist_left_back)
  {
    speed_up_left(0);
    speed_down_left(0);
    speed_up_right(180);
    speed_down_right(180);
  }
  else if (dist_left_forward < dist_left_back)
  {
    speed_up_left(180);
    speed_down_left(180);
    speed_up_right(0);
    speed_down_right(0);
  }
  else if (dist_left_forward == dist_left_back)
  {
    speed_up_left(180);
    speed_down_left(180);
    speed_up_right(180);
    speed_down_right(180);
  }
}
void read_light ()
{
  barrier = digitalRead(barrier_sensor_pin);
  dist_forward_down = d_forward_down.read();
  //Serial.println(barrier);
}
void fake_zone1()
{
  go_to_dist(22);
    wall_rovno();
    rotation_giro(-85);
    zad_rovno();
    go_to_dist(25);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_right();
    }
    go_to_dist(-1);
    rotation_giro(85);
    zad_rovno();
    read_distance();
    while (dist_right_forward < 30)
    {
      read_distance();
      go_forward_right();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(-85);
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
}
void fake_zone()
{
  read_mode();
  if (mode == 1)
  {
    go_to_dist(20);
    wall_rovno();
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(-85);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(-40);
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    go_to_dist(-15);
    rotation_giro(-85);
    zad_rovno();
    go_to_dist(25);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(-85);
    zad_rovno();
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(-85);
    delay(250);
    rotation_giro(-85);
    go_to_dist(20);
    read_knob_plat();
    while (knob_r_edge == 1)
    {
      read_knob_plat();
      go_forward_right();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(85);
    read_distance();
    while (dist_right_forward < 30)
    {
      read_distance();
      go_forward_right();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(85);
    go_to_dist(25);
    rotation_giro(85);
    zad_rovno();
    read_knob_plat();
    while ((knob_r_edge == 1 && knob_p_ru == 1) && knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_right();
      go_forward_left();
    }
    //pered_pos();
    go_to_dist(-1);
    rotation_giro(-85);
    pered_pos();
    go_to_dist(-5);
    rotation_giro(-85);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(170);
    zad_rovno();
    unload_start();
    delay(500);
    unload_end();
    delay(250);
    go_to_dist(-1);
    rotation_giro(85);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(-40);
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    ///////////////////////////////////круг
  }
  else if ( mode == 3)
  {
    go_to_dist(20);
    wall_rovno();
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(-85);
    zad_rovno();
    //    read_knob_plat();
    //    while (knob_l_edge == 1)
    //    {
    //      read_knob_plat();
    //      speed_up_left(160);
    //      speed_down_left(160);
    //      speed_up_right(160);
    //      speed_down_right(160);
    //    }
    //    go_to_dist(-1);
    //    rotation_giro(-40);
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    go_to_dist(-15);
    rotation_giro(-85);
    zad_rovno();
    go_to_dist(25);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(-85);
    read_knob_plat();
    while (knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(-40);
    //  zad_rovno();
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-2);
    rotation_giro(-85);
    delay(250);
    //rotation_giro(-85);
    // go_to_dist(20);
    read_knob_plat();
    while (knob_r_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(180);
    read_distance();
    while (dist_right_forward < 30)
    {
      read_distance();
      go_forward_right();
    }
    //pered_pos();
    go_to_dist(-15);
    rotation_giro(85);
    go_to_dist(25);
    rotation_giro(85);
    zad_rovno();
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_right();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(-85);
    //pered_pos();
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    go_to_dist(-15);
    rotation_giro(-85);
    zad_rovno();
    go_to_dist(20);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(170);
    zad_rovno();
    unload_start();
    delay(500);
    unload_end();
    delay(250);
    go_to_dist(-1);
    rotation_giro(-85);

    read_knob_plat();
    while (knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(40);
    read_distance();
    while (dist_right_forward < 30)
    {
      read_distance();
      go_forward_right();
    }
    pered_pos();
    go_to_dist(-1);
    rotation_giro(-85);
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    stop_motor();
  }
}
void gray_line()
{
  line_sensor();
  if (dataSensors[3] < limit_3_g && dataSensors[4] < limit_4_g && dataSensors[5] < limit_5_g && dataSensors[6] < limit_6_g)
  {
    fake_zone();
    //    write_map();
    //    go_to_zero();
    //    go_zigzag();
    //    vigruz();
  }
}
void stop_motor_special()
{
  speed_up_left(-200);
  speed_down_left(-200);
  speed_up_right(-200);
  speed_down_right(-200);
  delay(50);
  speed_up_left(0);
  speed_down_left(0);
  speed_up_right(0);
  speed_down_right(0);
}
void write_map()
{
  int val1 = 0;
  read_distance();
  read_distance();
  read_distance();
  while (dist_right_back > 20 || dist_left_back > 20)
  {
    read_distance();
    speed_up_left(200);
    speed_down_left(200);
    speed_up_right(200);
    speed_down_right(200);
  }

  go_to_dist(7);
  stop_motor_special();
  delay(200);
  wall_rovno();
  read_distance();
  read_distance();
  read_distance();
  read_distance();
  read_distance();
  read_distance();
  stop_motor();
  if (dist_middle_left <= 15)
  {
    if (dist_middle_right > limit_wall || dist_forward_down <= limit_wall)
    {
      val1 = 1;
      Map[5][0] = 1;
      Map[5][1] = 1;
    }
    if (dist_middle_right <= limit_wall || dist_forward_down > limit_wall)
    {
      val1 = 2;
      Map[0][0] = 1;
      Map[1][0] = 1;
    }
  }
  else if (dist_middle_right <= 15)
  {
    if (dist_middle_left > limit_wall || dist_forward_down <= limit_wall)
    {
      val1 = 3;
      Map[0][0] = 1;
      Map[0][1] = 1;
    }
    else if (dist_middle_left <= limit_wall || dist_forward_down > limit_wall)
    {
      val1 = 4;
      Map[5][0] = 1;
      Map[4][0] = 1;
    }
  }
  wall_go_r();
}
void wall_balance()
{

}
void wall_go_r()
{
  read_distance();
  read_knob_plat();
  while (dist_right_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    read_distance();
    read_knob_plat();
    go_forward_right();
  }
  stop_motor();
  read_knob_plat();
  stop_motor_special();
  read_distance();
  if (dist_forward_down < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[4][0] = 3;
      Map[5][0] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[5][6] = 3;
      Map[5][7] = 3;
    }
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    pered_pos();
  }
  else if (knob_l_edge == 0 || knob_r_edge == 0)
  {
    pered_pos();
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      if (Map[5][0] == 0)
      {
        read_distance();
        if (dist_middle_right > 10)
        {
          Map[4][0] = 2;
          Map[5][0] = 2;
          Map[5][1] = 2;
        }
      }
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      if (Map[5][7] == 0)
      {
        read_distance();
        if (dist_middle_right > 10)
        {
          //Serial.println("v 3");
          Map[4][7] = 2;
          Map[5][7] = 2;
          Map[5][6] = 2;
        }
      }
    }
  }
  else if (dist_forward_down < 60)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[2][0] = 3;
      Map[3][0] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[5][4] = 3;
      Map[5][5] = 3;
    }
    while (knob_l_edge == 1 && knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    pered_pos();
  }
  else if (dist_forward_down < 90)
  {
    if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[5][2] = 3;
      Map[5][3] = 3;
    }
    else if (Map[0][0] == 1 && Map[0][1] == 1)
    {

    }
    while (knob_l_edge == 1 && knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    pered_pos();
  }
  else if (dist_forward_down > 90)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      go_to_dist(-5);
      Map[5][0] = 3;
      Map[5][1] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      go_to_dist(-5);
      Map[4][7] = 3;
      Map[5][7] = 3;
    }
  }
  pered_pos();
  stop_motor_special();
  go_to_dist(-1);
  if (Map[0][1] == 1)
  {
    if ( Map[5][0] == 2)
    {
      rotation_giro(80);
      read_knob_plat();
      while (knob_l_edge == 1 && knob_r_edge == 1)
      {
        read_knob_plat();
        speed_up_left(160);
        speed_down_left(160);
        speed_up_right(160);
        speed_down_right(160);
      }
      stop_motor();
      go_to_dist(-2);
      rotation_giro(45);
      delay(250);
      wall_rovno();
    }
    else
    {
      rotation_giro(80);
      delay(250);
      wall_rovno();
    }
  }
  else if (Map[4][0] == 1)
  {
    if ( Map[5][7] == 2)
    {
      rotation_giro(80);
      delay(250);
      read_knob_plat();
      while (knob_l_edge == 1 && knob_r_edge == 1)
      {
        read_knob_plat();
        speed_up_left(160);
        speed_down_left(160);
        speed_up_right(160);
        speed_down_right(160);
      }
      stop_motor();
      go_to_dist(-2);
      rotation_giro(45);
    }
    else
    {
      rotation_giro(80);
      delay(250);
      wall_rovno();
    }
  }
  stop_motor();
  read_distance();
  read_knob_plat();
  stop_motor_special();
  while (dist_right_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    read_distance();
    read_knob_plat();
    go_forward_right();
  }
  stop_motor();
  read_distance();
  read_knob_plat();
  if (dist_forward_down < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[5][6] = 3;
      Map[5][7] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[0][7] = 3;
      Map[1][7] = 3;
    }
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    pered_pos();
  }
  else if (knob_l_edge == 0 || knob_r_edge == 0)
  {
    pered_pos();
    pered_pos();
    pered_pos();
    pered_pos();
    read_knob_plat();
    read_distance();
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      if (Map[5][7] == 0)
      {
        read_distance();
        if (dist_middle_right > 30)
        {
          Map[4][7] = 2;
          Map[5][7] = 2;
          Map[5][6] = 2;
        }
      }
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      if (Map[0][7] == 0)
      {
        pered_pos();
        read_distance();
        if (dist_middle_right > 15)
        {
          Map[0][7] = 2;
          Map[1][7] = 2;
          Map[0][6] = 2;
        }
      }
    }
  }
  else if (dist_forward_down < 60)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[5][5] = 3;
      Map[5][4] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[2][7] = 3;
      Map[3][7] = 3;
    }
    while (knob_l_edge == 1 && knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
  }
  else if (dist_forward_down < 90)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[5][2] = 3;
      Map[5][3] = 3;
    }
    while (knob_l_edge == 1 && knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
  }
  else if (dist_forward_down > 90)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      go_to_dist(-5);
      Map[4][7] = 3;
      Map[5][7] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      go_to_dist(-5);
      Map[0][7] = 3;
      Map[0][6] = 3;
    }
  }
  pered_pos();
  pered_pos();
  stop_motor_special();
  go_to_dist(-1);
  if (Map[0][1] == 1)
  {
    if ( Map[5][7] == 2)
    {
      rotation_giro(80);
      delay(250);
      read_knob_plat();
      while (knob_l_edge == 1 && knob_r_edge == 1)
      {
        read_knob_plat();
        speed_up_left(160);
        speed_down_left(160);
        speed_up_right(160);
        speed_down_right(160);
      }
      stop_motor();
      go_to_dist(-2);
      rotation_giro(45);
    }
    else
    {
      rotation_giro(80);
      delay(250);
      wall_rovno();
    }
  }
  else if (Map[4][0] == 1)
  {
    if ( Map[0][7] == 2)
    {
      rotation_giro(80);
      delay(250);
      read_knob_plat();
      while (knob_l_edge == 1 && knob_r_edge == 1)
      {
        read_knob_plat();
        speed_up_left(160);
        speed_down_left(160);
        speed_up_right(160);
        speed_down_right(160);
      }
      stop_motor();
      go_to_dist(-2);
      rotation_giro(45);
      delay(250);
    }
    else
    {
      rotation_giro(80);
      delay(250);
      wall_rovno();
    }
  }

  read_distance();
  read_knob_plat();
  while (dist_right_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    read_distance();
    read_knob_plat();
    go_forward_right();
  }
  stop_motor();
  read_distance();
  read_knob_plat();
  if (dist_forward_down < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[0][7] = 3;
      Map[1][7] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[0][0] = 3;
      Map[0][1] = 3;
    }
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
  }
  else if (knob_l_edge == 0 || knob_r_edge == 0)
  {
    read_knob_plat();
    pered_pos();
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      if (Map[5][7] == 0)
      {
        read_distance();
        if (dist_middle_right > 10)
        {
          Map[0][7] = 2;
          Map[1][7] = 2;
          Map[0][6] = 2;
        }
      }
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      if (Map[0][7] == 0)
      {
        read_distance();
        if (dist_middle_right > 10)
        {
          Map[0][0] = 2;
          Map[0][1] = 2;
          Map[1][0] = 2;
        }
      }
    }
  }
  else if (dist_forward_down < 60)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      Map[2][7] = 3;
      Map[3][7] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[0][2] = 3;
      Map[0][3] = 3;
    }
    while (knob_l_edge == 1 && knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
  }
  else if (dist_forward_down < 90)
  {
    if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      Map[0][4] = 3;
      Map[0][5] = 3;
    }
    while (knob_l_edge == 1 && knob_r_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
  }
  else if (dist_forward_down > 90)
  {
    if (Map[0][0] == 1 && Map[0][1] == 1)
    {
      go_to_dist(-5);
      Map[0][6] = 3;
      Map[0][7] = 3;
    }
    else if (Map[5][0] == 1 && Map[4][0] == 1)
    {
      go_to_dist(-5);
      Map[0][0] = 3;
      Map[0][1] = 3;
    }
  }
  pered_pos();
  stop_motor_special();
  go_to_dist(-1);
  if (Map[0][1] == 1)
  {
    if ( Map[0][7] == 2)
    {
      rotation_giro(80);
      delay(250);
      read_knob_plat();
      while (knob_l_edge == 1 && knob_r_edge == 1)
      {
        read_knob_plat();
        speed_up_left(160);
        speed_down_left(160);
        speed_up_right(160);
        speed_down_right(160);
      }
      stop_motor();
      go_to_dist(-2);
      rotation_giro(45);
      delay(250);
      wall_rovno();
    }
    else
    {
      rotation_giro(80);
    }
  }
  else if (Map[4][0] == 1)
  {
    if ( Map[0][0] == 2)
    {
      rotation_giro(80);
      read_knob_plat();
      while (knob_l_edge == 1 && knob_r_edge == 1)
      {
        read_knob_plat();
        speed_up_left(160);
        speed_down_left(160);
        speed_up_right(160);
        speed_down_right(160);
      }
      stop_motor();
      go_to_dist(-2);
      rotation_giro(45);
      delay(250);
    }
    else
    {
      rotation_giro(80);
      delay(250);
      wall_rovno();
    }
  }

  stop_motor();
  read_distance();
  read_knob_plat();
  stop_motor_special();
  read_knob_plat();
  read_distance();
  while (dist_right_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
  {
    read_distance();
    read_knob_plat();
    go_forward_right();
  }
  stop_motor();
  read_distance();
  read_knob_plat();
  if (knob_r_edge == 0 || knob_l_edge == 0)
  {
    if (Map[0][0] == 1)
    {
      rotation_giro(80);
      delay(250);
      wall_rovno();
      stop_motor();
    }
  }
  else
  {
    go_to_dist(15);
    rotation_giro(80);
    delay(250);
    wall_rovno();
    stop_motor();
  }
  Map_show();
}
void go_to_zero()
{
  if (Map[0][0] == 1 && Map[0][1] == 1)
  {
    if (Map[5][0] == 2)
    {}
    else if (Map[5][7] == 2)
    {
      rotation_giro(80);
      stop_motor();
    }
    else if ( Map[0][7] == 2)
    {
      read_knob_plat();
      read_distance();
      while (dist_right_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
      {
        read_distance();
        read_knob_plat();
        go_forward_right();
      }
      if (dist_right_forward > 30 && dist_forward_down > 30)
      {
        go_to_dist(-10);
        rotation_giro(80);
        zad_rovno();
      }
      else if (dist_right_forward > 30 && dist_forward_down < 30)
      {
        go_to_dist(15);
        rotation_giro(80);
      }
      else if (knob_r_edge == 0 || knob_l_edge == 0)
      {
        pered_pos();
        go_to_dist(-1);
        rotation_giro(80);
        zad_rovno();
      }
    }
    if (Map[0][0] == 1 && Map[1][0] == 1)
    {
      if ( Map[0][7] == 2)
      {}
      else if (Map[5][7] == 2)
      {}
      else if (Map[5][0] == 2)
      {}
    }
    if (Map[4][0] == 1 && Map[5][0] == 1)
    {
      if (Map[5][7] == 2)
      {
        rotation_giro(80);
        go_to_dist(30);
        read_knob_plat();
        read_distance();
        while (dist_left_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
        {
          read_distance();
          read_knob_plat();
          go_forward_left();
        }
        if (dist_left_forward > 30 && dist_forward_down > 30)
        {
          go_to_dist(-10);
          rotation_giro(-80);
          zad_rovno();
        }
        else if (dist_left_forward > 30 && dist_forward_down < 30)
        {
          go_to_dist(15);
          rotation_giro(-80);
        }
        else if (knob_r_edge == 0 || knob_l_edge == 0)
        {
          pered_pos();
          go_to_dist(-1);
          rotation_giro(-80);
          zad_rovno();
        }
      }
      else if ( Map[0][7] == 2)
      {
        read_knob_plat();
        read_distance();
        while (dist_right_forward < 30 && knob_r_edge == 1 && knob_l_edge == 1)
        {
          read_distance();
          read_knob_plat();
          go_forward_right();
        }
        pered_pos();
        go_to_dist(-1);
        rotation_giro(80);
        zad_rovno();
        stop_motor();
      }
    }
    else if (Map[0][0] == 2)
    {}
  }
  if (Map[5][0] == 1 && Map[5][1] == 1)
  {
    if (Map[0][0] == 2)
    {}
    else if ( Map[0][7] == 2)
    {}
    else if (Map[5][7] == 2)
    {}
  }
}
void go_zigzag()
{
  if (Map[0][7] == 2)
  {
    read_knob_plat();
    read_distance();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_distance();
      read_knob_plat();
      go_forward_right();
    }
    go_to_dist(-1);
    rotation_giro(80);
    go_to_dist(25);
    rotation_giro(80);
    zad_rovno();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_distance();
      read_knob_plat();
      go_forward_right();
      go_forward_left();
    }
    stop_motor();
    go_to_dist(-1);
    rotation_giro(-80);
    read_distance();
    while (dist_left_forward < 30)
    {
      read_distance();
      go_forward_left();
    }
    stop_motor();
    go_to_dist(-20);
    rotation_giro(-80);
    go_to_dist(30);
    read_knob_plat();
    read_distance();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_distance();
      read_knob_plat();
      go_forward_left();
    }
    pered_pos();
  }
  else if (Map[5][7] == 2)
  {
    go_to_dist(30);
    read_knob_plat();
    read_distance();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_distance();
      read_knob_plat();
      go_forward_left();
    }
    go_to_dist(-1);
    rotation_giro(-80);
    go_to_dist(25);
    rotation_giro(-80);
    zad_rovno();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_distance();
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    stop_motor();
    go_to_dist(-1);
    rotation_giro(80);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      read_distance();
      go_forward_right();
    }
    stop_motor();
    go_to_dist(-5);
    rotation_giro(80);
    read_knob_plat();
    read_distance();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_distance();
      read_knob_plat();
      go_forward_right();
    }
    pered_pos();
  }
}
void vigruz()
{
  if (Map[0][7] == 2)
  {
    go_to_dist(-1);
    rotation_giro(160);
    zad_rovno();
    unload_start();
    delay(1000);
    unload_end();
    delay(500);
  }
  else if (Map[5][7] == 2)
  {
    go_to_dist(-1);
    rotation_giro(-160);
    zad_rovno();
    unload_start();
    delay(500);
    shaking();
    delay(1000);
    unload_end();
    delay(500);
  }

}
void go_out()
{
  if (Map[5][7] == 2 && Map[0][1] == 3)
  {
    go_to_dist(1);
    rotation_giro(-80);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(45);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      go_forward_right();
    }
    go_to_dist(-1);
    rotation_giro(-80);
    read_distance();
    while (dist_left_middle < 20 )
    {
      read_distance();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    stop_motor();
  }
  if (Map[0][7] == 2 && Map[0][1] == 3)
  {

  }
  if (Map[5][7] == 2 && Map[4][0] == 3)
  {
    go_to_dist(-1);
    rotation_giro(80);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(-45);
    read_distance();
    while (dist_left_middle < 30)
    {
      read_distance();
      go_forward_left();
    }
    stop_motor();
  }
  if (Map[0][7] == 2 && Map[4][0] == 3)
  {
    go_to_dist(-1);
    rotation_giro(80);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(-45);
    read_knob_plat();
    while (knob_r_edge == 1 && knob_l_edge == 1)
    {
      read_knob_plat();
      speed_up_left(160);
      speed_down_left(160);
      speed_up_right(160);
      speed_down_right(160);
    }
    go_to_dist(-1);
    rotation_giro(-80);
    read_distance();
    while (dist_left_middle < 30)
    {
      read_distance();
      go_forward_left();
    }
    stop_motor();
  }
}
void Map_show()
{

}
void pered_pos()
{
  read_knob_plat();
  while ((knob_p_ru == 1 || knob_p_lu == 1) || (knob_r_edge == 1 || knob_l_edge == 1))
  {
    read_knob_plat();
    if (knob_p_ru == 1 && knob_p_lu == 1)
    {
      read_knob_plat();
      speed_up_left(200);
      speed_down_left(200);
      speed_up_right(200);
      speed_down_right(200);
    }
    else if (knob_p_ru == 0 && knob_p_lu == 1)
    {
      read_knob_plat();
      speed_up_left(250);
      speed_down_left(250);
      speed_up_right(-150);
      speed_down_right(-150);
    }
    else if (knob_p_ru == 1 && knob_p_lu == 0)
    {
      read_knob_plat();
      speed_up_left(-150);
      speed_down_left(-150);
      speed_up_right(250);
      speed_down_right(250);
    }
  }
  stop_motor();
}
void read_knob_edge()
{
  knob_r_edge = digitalRead(A0);
  knob_l_edge = digitalRead(A1);
}
void wall_rovno()
{
  read_distance();
  if (dist_middle_right < 30 || dist_middle_left < 30)
  {
    if (dist_middle_right > dist_middle_left)
    {
      read_distance();
      while (abs(dist_left_forward - dist_left_back) >= 2)
      {
        read_distance();
        if (dist_left_forward > dist_left_back)
        {
          speed_up_left(-130);
          speed_down_left(-130);
        }
        else if (dist_left_forward < dist_left_back)
        {
          speed_up_right(-130);
          speed_down_right(-130);
        }
      }
      stop_motor();
    }
    else if (dist_middle_right < dist_middle_left)
    {
      read_distance();
      while (  abs(dist_right_forward - dist_right_back) >= 2)
      {
        read_distance();
        if (dist_right_forward > dist_right_back)
        {
          speed_up_right(-130);
          speed_down_right(-130);
        }
        else if (dist_right_forward < dist_right_back)
        {
          speed_up_left(-130);
          speed_down_left(-130);
        }
      }
      stop_motor();
    }
  }
  stop_motor();
}
void zad_rovno()
{
  read_knob_plat();
  while ( knob_p_ld == 1 || knob_p_rd == 1)
  {
    read_knob_plat();
    if (knob_p_ld == 1 || knob_p_rd == 1)
    {
      speed_up_left(-200);
      speed_down_left(-200);
      speed_up_right(-200);
      speed_down_right(-200);
    }
    else if (knob_p_ld == 1 || knob_p_rd == 0)
    {
      read_knob_plat();
      speed_up_left(-150);
      speed_down_left(-150);
      speed_up_right(200);
      speed_down_right(200);
    }
    else if (knob_p_ld == 0 || knob_p_rd == 1)
    {
      read_knob_plat();
      speed_up_left(200);
      speed_down_left(200);
      speed_up_right(-150);
      speed_down_right(-150);
    }
  }
  stop_motor();
}
void rotation_giro (float angle_01)
{
  int koef = 1500 / 90;
  int vlue = koef * angle_01;
  if (angle_01 > 0)
  {
    enc_read_start();
    while ((enc_down_L + enc_down_R + enc_up_L + enc_up_R) / 4 < vlue)
    {
      speed_up_left(-200);
      speed_down_left(-200);
      speed_up_right(200);
      speed_down_right(200);
    }
    stop_motor();
  }
  else if (angle_01 < 0)
  {
    enc_read_start();
    while ((enc_down_L + enc_down_R + enc_up_L + enc_up_R) / 4 < -vlue)
    {
      speed_up_left(200);
      speed_down_left(200);
      speed_up_right(-200);
      speed_down_right(-200);
    }
    stop_motor();
  }
  enc_read_end();
}
void line_sensor ()
{
  dataSensors[1] = octoliner.analogRead(7);
  dataSensors[2] = octoliner.analogRead(6);
  dataSensors[3] = octoliner.analogRead(5);
  dataSensors[4] = octoliner.analogRead(4);
  dataSensors[5] = octoliner.analogRead(3);
  //dataSensors[6] = octoliner.analogRead(2);
  dataSensors[6] = dataSensors[5];
  dataSensors[7] = octoliner.analogRead(1);
  dataSensors[8] = octoliner.analogRead(0);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
  //Serial.println(dataSensors[1]);
}
void stop_motor()
{
  speed_up_left(0);
  speed_down_left(0);
  speed_up_right(0);
  speed_down_right(0);
}
void read_distance()
{
  dist_right_forward = d_forward_right.read();
  dist_right_back = d_backward_right.read();
  dist_left_forward = d_forward_left.read();
  dist_left_back = d_backward_left.read();
  //dist_middle_right = d_middle_right.read();
  //dist_middle_left = d_middle_left.read();
  //dist_forward_up = d_forward_up.read();
  //dist_forward_down = d_forward_down.read();
//      Serial.print("  dist_right_forward: ");
//      Serial.print(dist_right_forward);
//      Serial.print("  dist_right_back: ");
//      Serial.print(dist_right_back);
//      Serial.print("  dist_left_forward: ");
//      Serial.print(dist_left_forward);
//      Serial.print("  dist_left_back: ");
//      Serial.print(dist_left_back);
//      Serial.print("  dist_middle_right: ");
//      Serial.print(dist_middle_right);
//      Serial.print("  dist_middle_left: ");
//      Serial.print(dist_middle_left);
//      Serial.print("  dist_forward_up: ");
//      Serial.print(dist_forward_up);
//      Serial.print("  dist_forward_down: ");
//      Serial.println(dist_forward_down);
}
void serial_sensors()
{
  line_sensor();
  if (dataSensors[1] > limit_1)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[2] > limit_2)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[3] > limit_3)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[4] > limit_4)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[5] > limit_5)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[6] > limit_6)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[7] > limit_7)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  if (dataSensors[8] > limit_8)
    Serial.print(1);
  else
    Serial.print(0);
  Serial.print(" ");
  Serial.print("|");
  Serial.print(" ");
  Serial.print(limit_1);
  Serial.print(" ");
  Serial.print(limit_2);
  Serial.print(" ");
  Serial.print(limit_3);
  Serial.print(" ");
  Serial.print(limit_4);
  Serial.print(" ");
  Serial.print(limit_5);
  Serial.print(" ");
  Serial.print(limit_6);
  Serial.print(" ");
  Serial.print(limit_7);
  Serial.print(" ");
  Serial.print(limit_8);
  Serial.print(" ");
  Serial.print("|");
  Serial.print(" ");
  Serial.print(dataSensors[1]);
  Serial.print(" ");
  Serial.print(dataSensors[2]);
  Serial.print(" ");
  Serial.print(dataSensors[3]);
  Serial.print(" ");
  Serial.print(dataSensors[4]);
  Serial.print(" ");
  Serial.print(dataSensors[5]);
  Serial.print(" ");
  Serial.print(dataSensors[6]);
  Serial.print(" ");
  Serial.print(dataSensors[7]);
  Serial.print(" ");
  Serial.print(dataSensors[8]);
  Serial.println(" ");
}
void go_line_octoliner_old (int base_speed)
{
  int spd = base_speed;
  line_sensor();
  /////////////////

  if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|0|0|0|0|0|
  { 
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] > limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|1|0|0|0|0|
  {
    speed_up_left(spd / 1.5);
    speed_down_left(spd / 1.5);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] > limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|0|1|0|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(spd / 1 / 5);
    speed_down_right(spd / 1 / 5);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] > limit_3 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                         // |0|0|1|x|x|0|0|0|
  {
    speed_up_left(0);
    speed_down_left(0);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[6] > limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                        // |0|0|0|x|x|1|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(0);
    speed_down_right(0);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] > limit_2 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                                                                                // |0|1|x|x|x|x|0|0|
  {
    speed_up_left(-spd);
    speed_down_left(-spd);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[7] > limit_7 && dataSensors[8] < limit_8)                                                                                                               // |0|0|x|x|x|x|1|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(-spd);
    speed_down_right(-spd);
  }
  else if ( dataSensors[1] > limit_1 && dataSensors[8] < limit_8)                                                                                                                                                                       // |1|x|x|x|x|x|x|x|
  {
    speed_up_left(-(spd + 50));
    speed_down_left(-(spd + 50));
    speed_up_right(spd + 50);
    speed_down_right(spd + 50);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[8] > limit_8)                                                                                                                                                                      // |0|x|x|x|x|x|x|1|
  {
    speed_up_left(spd + 50);
    speed_down_left(spd + 50);
    speed_up_right(-(spd + 50));
    speed_down_right(-(spd + 50));
  }
}
void go_line_octoliner (int base_speed)
{
  int spd = base_speed;
  line_sensor();
  //gray_line();
  //event_from_camera();
  //  if ((dataSensors[1] > limit_1 && dataSensors[2] > limit_2 && dataSensors[3] > limit_3 && dataSensors[4] > limit_4) || (dataSensors[5] > limit_5 && dataSensors[6] > limit_6 && dataSensors[7] > limit_7 && dataSensors[8] > limit_8))
  //  {
  //    go_to_dist(3);
  //    line_sensor();
  //    if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
  //    {
  //      go_to_dist(-3);
  //    }
  //  }
  if ( dataSensors[1] > limit_1 && dataSensors[8] < limit_8)                                                                                                                                                                       // |1|x|x|x|x|x|x|x|
  {
    speed_up_left(-spd);
    speed_down_left(-spd);
    speed_up_right(spd + 50);
    speed_down_right(spd + 50);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[8] > limit_8)                                                                                                                                                                      // |0|x|x|x|x|x|x|1|
  {
    speed_up_left(spd + 50);
    speed_down_left(spd + 50);
    speed_up_right(-spd);
    speed_down_right(-spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] > limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|1|0|0|0|0|
  {
    speed_up_left(spd / 1.5);
    speed_down_left(spd / 1.5);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] > limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|0|1|0|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(spd / 1.5);
    speed_down_right(spd / 1.5);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] > limit_3 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                         // |0|0|1|x|x|0|0|0|
  {
    speed_up_left(0);
    speed_down_left(0);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[6] > limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                        // |0|0|0|x|x|1|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(0);
    speed_down_right(0);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] > limit_2 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                                                                                // |0|1|x|x|x|x|0|0|
  {
    speed_up_left(-spd);
    speed_down_left(-spd);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[7] > limit_7 && dataSensors[8] < limit_8)                                                                                                               // |0|0|x|x|x|x|1|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(-spd);
    speed_down_right(-spd);
  }


  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|0|0|0|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(spd);
    speed_down_right(spd);
  }
}
void go_line_octoliner1 (int base_speed)
{
  int spd = base_speed;
  line_sensor();
  //gray_line();
  //event_from_camera();
  //  if ((dataSensors[1] > limit_1 && dataSensors[2] > limit_2 && dataSensors[3] > limit_3 && dataSensors[4] > limit_4) || (dataSensors[5] > limit_5 && dataSensors[6] > limit_6 && dataSensors[7] > limit_7 && dataSensors[8] > limit_8))
  //  {
  //    go_to_dist(3);
  //    line_sensor();
  //    if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)
  //    {
  //      go_to_dist(-3);
  //    }
  //  }
  if ( dataSensors[1] > limit_1 && dataSensors[8] < limit_8)                                                                                                                                                                       // |1|x|x|x|x|x|x|x|
  {
    speed_up_left(-spd);
    speed_down_left(-spd);
    speed_up_right(spd + 50);
    speed_down_right(spd + 50);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[8] > limit_8)                                                                                                                                                                      // |0|x|x|x|x|x|x|1|
  {
    speed_up_left(spd + 50);
    speed_down_left(spd + 50);
    speed_up_right(-spd);
    speed_down_right(-spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] > limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|1|0|0|0|0|
  {
    speed_up_left(spd -30);
    speed_down_left(spd -30);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] > limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|0|1|0|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(spd -30);
    speed_down_right(spd -30);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] > limit_3 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                         // |0|0|1|x|x|0|0|0|
  {
    speed_up_left(100);
    speed_down_left(100);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[6] > limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                        // |0|0|0|x|x|1|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(100);
    speed_down_right(100);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] > limit_2 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8)                                                                                                                // |0|1|x|x|x|x|0|0|
  {
    speed_up_left(-spd);
    speed_down_left(-spd);
    speed_up_right(spd);
    speed_down_right(spd);
  }
  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[7] > limit_7 && dataSensors[8] < limit_8)                                                                                                               // |0|0|x|x|x|x|1|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(-spd);
    speed_down_right(-spd);
  }


  else if ( dataSensors[1] < limit_1 && dataSensors[2] < limit_2 && dataSensors[3] < limit_3 && dataSensors[4] < limit_4 && dataSensors[5] < limit_5 && dataSensors[6] < limit_6 && dataSensors[7] < limit_7 && dataSensors[8] < limit_8) // |0|0|0|0|0|0|0|0|
  {
    speed_up_left(spd);
    speed_down_left(spd);
    speed_up_right(spd);
    speed_down_right(spd);
  }
}
void read_knob_plat()
{
  knob_p_ru = digitalRead(knob_plat_ru);
  knob_p_rd = digitalRead(knob_plat_rd);
  knob_p_lu = digitalRead(knob_plat_lu);
  knob_p_ld = digitalRead(knob_plat_ld);
  knob_r_edge = digitalRead(A7);
  knob_l_edge = digitalRead(A6);
}
void read_mode()
{
  knob_pm_1 = digitalRead(knob_plat_mode_1);
  knob_pm_2 = digitalRead(knob_plat_mode_2);
  if (knob_pm_1 == 0 && knob_pm_2 == 1)
    mode = 1;
  else if (knob_pm_1 == 1 && knob_pm_2 == 1)
    mode = 2;
  else if (knob_pm_1 == 1 && knob_pm_2 == 0)
    mode = 3;
}
void speed_up_right (int speed_3)
{
  //event_from_camera();
  int speed_wheel_3 = speed_3;
  if (speed_wheel_3 > 0)
  {
    digitalWrite(in3_Right, HIGH);
    digitalWrite(in4_Right, LOW);
    analogWrite(enB_Right, speed_wheel_3);
  }
  else if (speed_wheel_3 < 0)
  {
    digitalWrite(in3_Right, LOW);
    digitalWrite(in4_Right, HIGH);
    analogWrite(enB_Right, -speed_wheel_3);
  }
  else
  {
    digitalWrite(in3_Right, LOW);
    digitalWrite(in4_Right, LOW);
    analogWrite(enB_Right, 0);
  }
}

void speed_down_right (int speed_4)
{
  int speed_wheel_4 = speed_4;
  if (speed_wheel_4 > 0)
  {
    digitalWrite(in1_Right, HIGH);
    digitalWrite(in2_Right, LOW);
    analogWrite(enA_Right, speed_wheel_4);
  }
  else if (speed_wheel_4 < 0)
  {
    digitalWrite(in1_Right, LOW);
    digitalWrite(in2_Right, HIGH);
    analogWrite(enA_Right, -speed_wheel_4);
  }
  else
  {
    digitalWrite(in1_Right, LOW);
    digitalWrite(in2_Right, LOW);
    analogWrite(enA_Right, 0);
  }
}
void speed_up_left (int speed_1)
{
  int speed_wheel_1 = speed_1;
  if (speed_wheel_1 > 0)
  {
    digitalWrite(in3_Left, HIGH);
    digitalWrite(in4_Left, LOW);
    analogWrite(enB_Left, speed_wheel_1);
  }
  else if (speed_wheel_1 < 0)
  {
    digitalWrite(in3_Left, LOW);
    digitalWrite(in4_Left, HIGH);
    analogWrite(enB_Left, -speed_wheel_1);
  }
  else
  {
    digitalWrite(in3_Left, LOW);
    digitalWrite(in4_Left, LOW);
    analogWrite(enB_Left, 0);
  }
}
void speed_down_left (int speed_2)
{
  int speed_wheel_2 = speed_2;
  if (speed_wheel_2 > 0)
  {
    digitalWrite(in1_Left, HIGH);
    digitalWrite(in2_Left, LOW);
    analogWrite(enA_Left, speed_wheel_2);
  }
  else if (speed_wheel_2 < 0)
  {
    digitalWrite(in1_Left, LOW);
    digitalWrite(in2_Left, HIGH);
    analogWrite(enA_Left, -speed_wheel_2);
  }
  else
  {
    digitalWrite(in1_Left, LOW);
    digitalWrite(in2_Left, LOW);
    analogWrite(enA_Left, 0);
  }
}
void led_stop()
{
  pixels_left->clear();
  pixels_right->clear();
  pixels_left->show();
  pixels_right->show();
}
void led_marker(int s1, int s2, int s3)
{
  pixels_left->clear();
  pixels_left->setPixelColor(0, pixels_left->Color(s1, s2, s3));
  pixels_left->show();
}
void set_color_left(int s1, int s2, int s3)
{
  pixels_left->clear();
  pixels_left->setPixelColor(0, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(1, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(2, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(3, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(4, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(5, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(6, pixels_left->Color(s1, s2, s3));
  pixels_left->setPixelColor(7, pixels_left->Color(s1, s2, s3));
  pixels_left->show();
}
void set_color_right(int s1, int s2, int s3)
{
  pixels_right->clear();
  pixels_right->setPixelColor(0, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(1, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(2, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(3, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(4, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(5, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(6, pixels_right->Color(s1, s2, s3));
  pixels_right->setPixelColor(7, pixels_right->Color(s1, s2, s3));
  pixels_right->show();
}
void rainbow()
{
  pixels_left->clear();
  pixels_right->clear();
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(0, pixels_left->Color(255, 0, 0));
  pixels_right->setPixelColor(0, pixels_right->Color(255, 0, 0));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(1, pixels_left->Color(255, 127, 0));
  pixels_right->setPixelColor(1, pixels_right->Color(255, 127, 0));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(2, pixels_left->Color(255, 255, 0));
  pixels_right->setPixelColor(2, pixels_right->Color(255, 255, 0));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(3, pixels_left->Color(0, 255, 0));
  pixels_right->setPixelColor(3, pixels_right->Color(0, 255, 0));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(4, pixels_left->Color(0, 255, 255));
  pixels_right->setPixelColor(4, pixels_right->Color(0, 255, 255));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(5, pixels_left->Color(0, 127, 255));
  pixels_right->setPixelColor(5, pixels_right->Color(0, 127, 255));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(6, pixels_left->Color(0, 0, 255));
  pixels_right->setPixelColor(6, pixels_right->Color(0, 0, 255));
  pixels_left->show();
  pixels_right->show();
  delay(250);
  pixels_left->setPixelColor(7, pixels_left->Color(127, 0, 255));
  pixels_right->setPixelColor(7, pixels_right->Color(127, 0, 255));
  pixels_left->show();
  pixels_right->show();
  delay(250);
}
void led_test()
{
  set_color_right(255, 255, 255);
  set_color_left(255, 255, 255);
  delay(1000);
  set_color_right(255, 0, 0);
  set_color_left(255, 0, 0);
  delay(1000);
  set_color_right(255, 255, 0);
  set_color_left(255, 255, 0);
  delay(1000);
  set_color_right(0, 255, 0);
  set_color_left(0, 255, 0);
  delay(1000);
  set_color_right(0, 0, 255);
  set_color_left(0, 0, 255);
  delay(1000);
}
void unload_start ()
{
  myservo_4.write(entry_open_angle);
  delay(250);
  myservo_3.write(basket_up_angle);
  delay(250);
  myservo_3.write(basket_down_angle);
  delay(250);
  myservo_3.write(basket_up_angle);
  delay(250);
  myservo_3.write(basket_down_angle);
  delay(250);
  myservo_3.write(basket_up_angle);
  delay(250);
  myservo_3.write(basket_down_angle);
  delay(250);
}
void unload_end ()
{
  myservo_4.write(entry_close_angle);
  delay(250);
}
void shaking ()
{
  myservo_4.write(entry_middle_angle);
  delay(200);
  myservo_3.write(basket_up_angle);
  delay(250);
  myservo_3.write(basket_down_angle);
  delay(250);
  myservo_3.write(basket_up_angle);
  delay(250);
  myservo_3.write(basket_down_angle);
  delay(250);
  myservo_3.write(basket_up_angle);
  delay(250);
  myservo_3.write(basket_down_angle);
  delay(250);
  myservo_4.write(entry_close_angle);
  delay(200);
}
void hvat_up ()
{
  myservo_2.write(hvat_close_angle);
  delay(500);
  myservo_1.write(hvat_up_angle);
  delay(500);
  myservo_2.write(hvat_close_angle - 20);
  delay(200);
  myservo_2.write(hvat_close_angle);
}
void hvat_down ()
{
  myservo_2.write(hvat_open_angle);
  myservo_1.write(hvat_down_angle);
  delay(500);
}
void enc_read_serial()
{
  enc_read_start();
}
void enc_read_start()
{
  attachInterrupt(encLU, enc_up_left, CHANGE);
  attachInterrupt(encLD, enc_down_left, CHANGE);
  attachInterrupt(encRU, enc_up_right, CHANGE);
  attachInterrupt(encRD, enc_down_right, CHANGE);
}
void enc_read_end()
{
  detachInterrupt(encLU);
  detachInterrupt(encLD);
  detachInterrupt(encRU);
  detachInterrupt(encRD);
  enc_up_L = 0;
  enc_down_L = 0;
  enc_up_R = 0;
  enc_down_R = 0;
}
void enc_up_left ()
{
  enc_up_L++;
}
void enc_down_left ()
{
  enc_down_L++;
}
void enc_up_right ()
{
  enc_up_R++;
}
void enc_down_right ()
{
  enc_down_R++;
}
void all_speed_pid_LU_LD_RU_RD(float lluu, float lldd, float rruu, float rrdd)
{
  speed_up_right(speed_PID_RU);
  speed_down_right(speed_PID_RD);
  speed_up_left(speed_PID_LU);
  speed_down_left(speed_PID_LD);
  attachInterrupt(encLU, enc_up_left, CHANGE);
  attachInterrupt(encLD, enc_down_left, CHANGE);
  attachInterrupt(encRU, enc_up_right, CHANGE);
  attachInterrupt(encRD, enc_down_right, CHANGE);
  if (millis() - timer >= time_delay)
  {
    detachInterrupt(encLU);
    detachInterrupt(encLD);
    detachInterrupt(encRU);
    detachInterrupt(encRD);
    real_speed_LU = ((enc_up_L * cm_in_1_tik) / time_delay * 1000) * k_pwm_cm;
    real_speed_LD = ((enc_down_L * cm_in_1_tik) / time_delay * 1000) * k_pwm_cm;
    real_speed_RU = ((enc_up_R * cm_in_1_tik) / time_delay * 1000) * k_pwm_cm;
    real_speed_RD = ((enc_down_R * cm_in_1_tik) / time_delay * 1000) * k_pwm_cm;
    speed_PID_LU = computePID_LU(real_speed_LU, lluu, (time_delay / 1000), kp1, ki1, kd1, -255, 255);
    speed_PID_LD = computePID_LD(real_speed_LD, lldd, (time_delay / 1000), kp2, ki2, kd2, -255, 255);
    speed_PID_RU = computePID_RU(real_speed_RU, rruu, (time_delay / 1000), kp3, ki3, kd3, -255, 255);
    speed_PID_RD = computePID_RD(real_speed_RD, rrdd, (time_delay / 1000), kp4, ki4, kd4, -255, 255);
    speed_PID_RU = speed_PID_RU;
    speed_PID_RD = speed_PID_RD;
    speed_PID_LU = speed_PID_LU;
    speed_PID_LD = speed_PID_LD;

    enc_up_L = 0;
    enc_down_L = 0;
    enc_up_R = 0;
    enc_down_R = 0;
    timer = millis();
  }
}
void skip_barrier (int varr) // 1=left, 2=right, 3=random
{
  int varrr = varr;
  int sppeed = 150;
  read_distance();
  if (dist_forward_down <= 5)
  {
    stop_motor();
    read_distance();
    go_to_dist(-5);
    if (varrr == 3)
    {
      if (dist_right_middle > dist_left_middle) //right
      {
        rotation_giro(-85);
        read_distance();
        while (dist_left_back < 20)
        {
          read_distance();
          speed_up_right(sppeed);
          speed_up_left(sppeed);
          speed_down_right(sppeed);
          speed_down_left(sppeed);
        }
        stop_motor();
m1:
        go_to_dist(20);
        rotation_giro(85);
        read_distance();
        line_sensor();
        while (dist_left_back > 20 && dataSensors[1] < limit_1)
        {
          read_distance();
          line_sensor();
          speed_up_right(sppeed);
          speed_up_left(sppeed);
          speed_down_right(sppeed);
          speed_down_left(sppeed);
        }
        if (dist_left_back < 20)
        {
          goto m1;
        }
        else if (dataSensors[1] > limit_1)
        {
          rotation_giro(-85);
          go_to_dist(-5);
        }
      }
      else if (dist_left_middle > dist_right_middle) //left
      {
        rotation_giro(88);
        read_distance();
        while (dist_right_back < 20)
        {
          read_distance();
          speed_up_right(sppeed);
          speed_up_left(sppeed);
          speed_down_right(sppeed);
          speed_down_left(sppeed);
        }
        stop_motor();
m2:
        go_to_dist(15);
        rotation_giro(-88);
        read_distance();
        line_sensor();
        while (dist_right_back > 20 && dataSensors[8] < limit_8)
        {
          read_distance();
          line_sensor();
          speed_up_right(sppeed);
          speed_up_left(sppeed);
          speed_down_right(sppeed);
          speed_down_left(sppeed);
        }
        if (dist_right_back < 20)
        {
          goto m2;
        }
        else if (dataSensors[8] > limit_8)
        {
          rotation_giro(-88);
          go_to_dist(-5);
        }
      }
    }
    else if (varrr == 1)
    {
      rotation_giro(88);
      read_distance();
      while (dist_right_back < 20)
      {
        read_distance();
        speed_up_right(sppeed);
        speed_up_left(sppeed);
        speed_down_right(sppeed);
        speed_down_left(sppeed);
      }
      stop_motor();
m3:
      go_to_dist(15);
      rotation_giro(-88);
      read_distance();
      line_sensor();
      while (dist_right_back > 20 && dataSensors[8] < limit_8)
      {
        read_distance();
        line_sensor();
        speed_up_right(sppeed);
        speed_up_left(sppeed);
        speed_down_right(sppeed);
        speed_down_left(sppeed);
      }
      if (dist_right_back < 20)
      {
        goto m3;
      }
      else if (dataSensors[8] > limit_8)
      {
        rotation_giro(88);
        go_to_dist(-5);
      }
    }
    else if (varrr == 2)
    {
      rotation_giro(-85);
      read_distance();
      while (dist_left_back > 20)
      {
        read_distance();
        speed_up_right(sppeed);
        speed_up_left(sppeed);
        speed_down_right(sppeed);
        speed_down_left(sppeed);
      }
      stop_motor();
m4:
      go_to_dist(15);
      rotation_giro(85);
      read_distance();
      line_sensor();
      while (dist_left_back > 15 && dataSensors[1] < limit_1)
      {
        read_distance();
        line_sensor();
        speed_up_right(sppeed);
        speed_up_left(sppeed);
        speed_down_right(sppeed);
        speed_down_left(sppeed);
      }
      if (dist_left_back < 15)
      {
        goto m4;
      }
      else if (dataSensors[1] > limit_1)
      {
        rotation_giro(-85);
        go_to_dist(-5);
      }
    }
    stop_motor();
  }
}
void go_to_dist(int dist1)
{
  int speeed = 200;
  int dist11 = dist1 / cm_in_1_tik;
  enc_read_start();
  if (dist1 > 0)
  {
    while (((enc_up_L + enc_down_L + enc_up_R + enc_down_R) / 4) < abs(dist11))
    {
      speed_up_right(speeed);
      speed_up_left(speeed);
      speed_down_right(speeed);
      speed_down_left(speeed);
    }
    stop_motor();
  }
  else if (dist1 < 0)
  {
    while (((enc_up_L + enc_down_L + enc_up_R + enc_down_R) / 4) < abs(dist11))
    {
      speed_up_right(-speeed);
      speed_up_left(-speeed);
      speed_down_right(-speeed);
      speed_down_left(-speeed);
    }
  }
  enc_read_end();
  stop_motor();
}
