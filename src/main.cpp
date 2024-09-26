#include <Arduino.h>
#include <ESP32Servo.h>
#include <Motor.h>

/*足周り*/
int16_t front_left = 0;
int16_t front_right = 0;
int16_t rear_left = 0;
int16_t rear_right = 0;

const uint16_t ROBOT_RADIUS = 1; // ロボットの中心からオムニまでの長さ
const uint8_t DEAD_ZONE = 30; // デッドゾーン

const uint32_t TIMEOUT = 10000;  // タイムアウト時間(ms)
uint32_t lastReceiveTime = 0; // 最後にデータが送られてきた時間

/*MotorPIN*/
Motor frontLeft(32, 22, 3); // motor1
Motor frontRight(33, 21, 4);
Motor rearLeft(25, 19, 5);
Motor rearRight(14, 26, 6);
Motor collectMotor(18, 5, 7); // motor5
/*--------------*/

bool is_auto_mode = false; // ボール回収

/*Serial*/
const uint8_t START_BYTE = 0x02;
const uint8_t END_BYTE = 0x03;
const uint8_t BUFFER_LEN = 10;

bool is_start = false;
bool is_next_bytes_data = false;
uint8_t bytes = 0;
uint8_t current_bytes = 0;
int8_t buffer[BUFFER_LEN];
/*------------*/

/*Servo*/
Servo RightLaunch;
Servo LeftLaunch;

const uint8_t L_LAUNCH_SERVO_PIN = 23;
const uint8_t R_LAUNCH_SERVO_PIN = 27;

const uint8_t L_SET_DEGREE = 180;
const uint8_t L_LAUNCHING_DEGREE = 90;
const uint8_t R_SET_DEGREE = 90;
const uint8_t R_LAUNCHING_DEGREE = 180;

bool r_launched = false;
bool l_launched = false;
/*-------------*/

struct ControlData {
  int8_t left_x; // 左スティックX
  int8_t left_y; // 左スティックY
  uint8_t flags; //
};

/*flagsの中身*/
bool collect_flag = 0;
bool r_launch_flag = 0;
bool l_launch_flag = 0;
bool left_flag = 0;
bool right_flag = 0;

/*オムニ*/
void calculateWheelRPMs(int16_t x, int16_t y, bool l, bool r) {
  int8_t rotation = 0;

  if (l && r) {
    rotation = 0; // 両方が押された場合は回転なしにする
  } else if (l) {
    rotation = -64;
  } else if (r) {
    rotation = 64;
  }

  front_right = (-x + y) / sqrt(2.0) - rotation * ROBOT_RADIUS;
  front_left = (x + y) / sqrt(2.0) + rotation * ROBOT_RADIUS;
  rear_right = (x - y) / sqrt(2.0) - rotation * ROBOT_RADIUS;
  rear_left = (-x - y) / sqrt(2.0) + rotation * ROBOT_RADIUS;
}

void runMotor(int16_t motor_value, Motor &motor, bool reverse_dir) {
  motor_value = (abs(motor_value) > DEAD_ZONE ? motor_value : 0);
  motor_value = (abs(motor_value) > 255 ? 255 : motor_value);
  bool direction = motor_value < 0 ? reverse_dir : !reverse_dir;
  motor.run(abs(motor_value), direction);
}

/*debug*/
void printControlData(const ControlData &data) {
  Serial.println("Control Data:");
  Serial.print("Left Stick X: ");
  Serial.println(data.left_x);
  Serial.print("Left Stick Y: ");
  Serial.println(data.left_y);

  Serial.print("collect_flag: ");
  Serial.println(collect_flag ? "Pressed" : "Not Pressed");
  Serial.print("Right_Launching: ");
  Serial.println(r_launch_flag ? "Pressed" : "Not Pressed");
  Serial.print("Left_Launching: ");
  Serial.println(l_launch_flag ? "Pressed" : "Not Pressed");

  Serial.print("Left: ");
  Serial.println(left_flag ? "Pressed" : "Not Pressed");
  Serial.print("Right: ");
  Serial.println(right_flag ? "Pressed" : "Not Pressed");
}
/*--------------*/

/*シリアルをリセット*/
void serialReset() {
  is_start = false;
  is_next_bytes_data = false;
  bytes = 0;
  current_bytes = 0;
  for (int i = 0; i < BUFFER_LEN; i++) {
    buffer[i] = 0;
  }
}

/*MotorStop*/
void motorSafety(){ 
    frontLeft.run(0, 0);
    frontRight.run(0, 0);
    rearLeft.run(0, 0);
    rearRight.run(0, 0);
    collectMotor.run(0,0);
    RightLaunch.write(R_SET_DEGREE);
    LeftLaunch.write(L_SET_DEGREE);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  RightLaunch.attach(L_LAUNCH_SERVO_PIN);
  LeftLaunch.attach(R_LAUNCH_SERVO_PIN);

  RightLaunch.write(R_SET_DEGREE);
  LeftLaunch.write(L_SET_DEGREE);
}

void loop() {
  // timeout
  
  if (is_start && millis() - lastReceiveTime > TIMEOUT) {
    serialReset();
    motorSafety();
  }

  if (Serial2.available()) {
    lastReceiveTime = millis();
    char data = Serial2.read();
    if (!is_start && !is_next_bytes_data && data == START_BYTE) {
      Serial.println("Start");//Serial受信開始
      is_start = true;
      is_next_bytes_data = true;
    } else if (is_start && is_next_bytes_data) {
      bytes = data;
      Serial.printf("Bytes: %d\n", bytes);
      is_next_bytes_data = false;
    } else if (is_start && !is_next_bytes_data && current_bytes < bytes) {
      Serial.printf("Data: %02x\n", data);
      buffer[current_bytes] = data;
      current_bytes += 1;
    } else if (is_start && !is_next_bytes_data && current_bytes == bytes &&
               data == END_BYTE) {
      Serial.println("End");  //serial終了
      for (int i = 0; i < 3; i++) {
        Serial.printf("Buffer: %02x\n", buffer[i] & 0x000000FF);
      }
      ControlData ps;

      // 構造体にコピー
      memcpy(&ps.left_x, &buffer[0], 1); // left_x
      memcpy(&ps.left_y, &buffer[1], 1); // right_x

      ps.flags = buffer[2]; // buffer[2] から 8ビットのデータを flags にコピー

      collect_flag = (ps.flags & 0b10000000) > 0;  // 回収モータ
      r_launch_flag = (ps.flags & 0b01000000) > 0; // 右発射
      l_launch_flag = (ps.flags & 0b00100000) > 0; // 左発射
      left_flag = (ps.flags & 0b00010000) > 0;     // 左回転
      right_flag = (ps.flags & 0b00001000) > 0;    // 右回転
      serialReset();

      printControlData(ps); // debug用
      calculateWheelRPMs(ps.left_x, ps.left_y, left_flag, right_flag);

      runMotor(front_left, frontLeft, 1);   // Front left motor
      runMotor(front_right, frontRight, 0); // Front right motor
      runMotor(rear_left, rearLeft, 1);     // Rear left motor
      runMotor(rear_right, rearRight, 0);   // Rear right motor*/

      if (l_launch_flag) {  //左発射
        if (!l_launched) {
          LeftLaunch.write(L_LAUNCHING_DEGREE);
        } else {
          LeftLaunch.write(L_SET_DEGREE);
        }
        l_launched = !l_launched;
      }

      if (r_launch_flag) {  //右発射
        if (!r_launched) {
          RightLaunch.write(R_LAUNCHING_DEGREE);
        } else {
          RightLaunch.write(R_SET_DEGREE);
        }
        r_launched = !r_launched;
      }

      if (collect_flag) { //ボール回収フラグ
        is_auto_mode = !is_auto_mode;
      }
      if (is_auto_mode) {
        collectMotor.run(64, 0); // モーターを回し続ける
      }
      else{
        collectMotor.run(0,0);
      }
    } else {
      serialReset();
      motorSafety();
    }
  }
}