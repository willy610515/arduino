/*
  四足機器人arduino 草稿碼
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.

  [source] This is the main file that manages [kinematics] & [hardware]
  all the important parameters are set in this file.

  Comment Description:

  /// comment 留言

  //> used to explain the function of a line 用來解釋函式中的某行程式碼
  //: used to summurize the function of multiple lines 用來總結函式中的多行程式碼

  === used for headers 用於標題
  ::: used for sketch parts 用於草稿碼部件

  // ## used to explain the measurement unit of a variable 用來解釋一個變量的度量單位
  // !! used for warnings 用於警告
*/

#include "datatypes.h"

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*#include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
  MPU6050 mpu;*/

#include <PS4Controller.h> //PS4搖桿函式庫

/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//> stores the frequency of the loop function 此變數儲存loop函式的頻率
const float frequency = 440.0; // ## Hz

/// Kinematics Parameters 運動學參數

//: stores the location, rotation and scale of the main [body]儲存主體 [body] 的位置、旋轉和縮放
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## {mm, mm, mm}
  {0, 0, 0},   // ## {deg, deg, deg}
  {300, 40, 180} // ## {mm, mm, mm}
};

//: stores the parent joint location relative to the [body] 儲存相對於 [body] 的父關節位置
const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { -50, 0, 0}  // ## {mm, mm, mm}
};
const float bone_length = 105; // ## mm

//: high level parameters for the step function 階躍函數的高級參數
const datatypes::Vector step_extent = {40, 40, 26}; // ## {mm, mm}
float vrt_offset = - 16.50; // ## mm //垂直
float hrz_offset = - 6.00; // ## mm //水平

float base_offset[] = { 0, -1, 0, -2};
const float precision = 0.001; // ## mm

void setup() {
  Serial.begin(115200);

  init_hardware();
  init_input();
}

//: those local variables control the step direction and period 那些局部變量控制步伐方向和周期
datatypes::Vector2D _direction = {0, 0};
float turn = 0; //> indicates the direction of rotation 表示旋轉方向
float height = 0; //> indicates the leg extension 表示腿部伸展

int state = 0; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection 指示步態的類型，（0）空閒，（1）小跑，（2）偏航，（3）俯仰滾動，（4）物體檢測
float _period = 10.0; //> indicates the number of steps every second 表示每秒的步數

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body 這個變量存儲了body的相對旋轉

unsigned long duration;
int sample_sum, sample_num = 10,
                sample_index;
float freq;

void loop() {
  duration = millis(); //經過時間 // millis() 函式會回傳 Arduino 從開始執行程式一直到目前為止的千分之一秒數值(number of milliseconds)

  handle_hardware();
  handle_kinematics(_direction, turn, height, _period);

  handle_input();

  if (Serial.available())
    handle_serial();

  // this code gets the frequency of the loop function 此程式碼獲取loop函數的頻率
  /*sample_sum += 1000.0 / (millis() - duration);
    sample_index++;

    if (sample_index > sample_num) {
    freq = sample_sum / sample_num;
    Serial.println(freq);
    sample_sum = 0;
    sample_index = 0;
    }*/
}

float vo, ho;
void init_input() {
  PS4.begin("F8:C3:9E:3F:F8:10"); // !! replace with your own DualShock4 Controller Bluetooth MAC address 替換為您自己的 DualShock4 控制器藍牙 MAC 地址
  vo = vrt_offset;
  ho = hrz_offset;
}

bool _tb = false;
float stick_min = 6.f;
float lx, ly, rx, ry;
void handle_input() {
  if (PS4.isConnected()) {
    lx = inter(lx, PS4.data.analog.stick.lx / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick 獲得左模擬搖桿的x插值位置
    ly = inter(ly, PS4.data.analog.stick.ly / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick 獲得左模擬搖桿的y插值位置
    rx = inter(rx, PS4.data.analog.stick.rx / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick 獲得右模擬搖桿的x插值位置
    ry = inter(ry, PS4.data.analog.stick.ry / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick 獲得右模擬搖桿的y插值位置

    if (abs(lx) > stick_min) { //> checks whether the stick position is out of the deadzone 確認搖桿lx是否位於死區之外
      float x0 = lx - stick_min * sign(lx); //> subtracts the deadzone 減去死區
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> checks whether the stick position is out of the deadzone 確認搖桿ly是否位於死區之外
      float y0 = ly - stick_min * sign(ly); //> subtracts the deadzone 減去死區
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> checks whether the stick position is out of the deadzone 確認搖桿rx是否位於死區之外
      float x1 = rx - stick_min * sign(rx); //> subtracts the deadzone 減去死區
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> checks whether the stick position is out of the deadzone 確認搖桿ry是否位於死區之外
      float y1 = ry - stick_min * sign(ry); //> subtracts the deadzone 減去死區
      height = y1;
    } else height = 0;
  }

  if (PS4.data.button.touchpad) { //> checks the touchpad state 檢察觸控板狀態
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;
    }
  } else _tb = true;
}

// !! make sure you have enabled Newline or Carriage return 確認你是否已經啟用換行或是回車
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input (0）用于校準和測試，（1）使用序列作為輸入
void handle_serial() {
  //: reads and stores the serial data 讀取和儲存序列阜資料
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 13 || c == 32 || c == '\n') {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0)
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

//: this is an interpolation function used to smooth 這是一個內插函數，用於使平滑
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) {
  //: properties 0 is used to calibrate the joints 「properties 0」用來校準接頭
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo);
  }
  //: properties 1 is used for small adjustments to balance the weight 「properties 1」用來小幅調整以平衡重量
  else if (properties == 1) {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount;
  }
}
