#include <Arduino.h>
#include <CAN.h>
#include <ESP32Servo.h>

const unsigned int LENGTH = 1; // ロボットの中心からオムニまでの長さ
const unsigned int ID = 0x555; // ID

const uint8_t DEAD_ZONE = 30; // デッドゾーン

const uint32_t TIMEOUT = 50;  // タイムアウト時間(ms)
uint32_t lastReceiveTime = 0; // 最後にデータが送られてきた時間

const uint8_t TX_PIN = 17;
const uint8_t RX_PIN = 16;

const uint8_t RIGHT_SERVO_PIN = 23; // 右発射用サーボ
const uint8_t LEFT_SERVO_PIN = 27;  // 左発射用サーボ

const uint8_t COLLECT_PWM = 32; // 回収用モータPWM
const uint8_t COLLECT_DIR = 22;  // DIR
//const uint8_t TAKE_PWM = 33;   // PWM
//const uint8_t TAKE_DIR = 21;   // DIR

/*Servo*/
Servo leftLaunchingServo;
Servo rightLaunchingServo;

uint16_t left_launching_degree = 0;
uint16_t right_launching_degree = 0;

struct ControlData {
  /*25bit*/
  int8_t left_x; // 左スティックX  -128~128の範囲 8bit
  int8_t left_y; // 左スティックY -128~128の範囲

  uint8_t catching : 1;    //  ボール回収 0-1 1bit
  uint8_t r_launching : 1; // 右発射 0-1
  uint8_t l_launching : 1; // 左発射  0-1

  uint8_t left : 1;  // 左旋回 0-1
  uint8_t right : 1; // 右旋回 0-1
};

struct Motor_RPMs {
  uint16_t frontLeft;
  uint16_t frontRight;
  uint16_t rearLeft;
  uint16_t rearRight;
};

Motor_RPMs calculateWheelRPMs(int x, int y, bool l,
                              bool r) { // オムニホイールの各RPMを代入
  Motor_RPMs RPMs;
  int8_t rotation = 0;

  rotation = l ? -64 : 0;
  rotation = r ? 64 : 0;

  RPMs.frontLeft =
      (cos(radians(45.0)) * x) + (sin(radians(45.0)) * y) + rotation;
  RPMs.frontRight =
      (cos(radians(135.0)) * x) + (sin(radians(135.0)) * y) - rotation;
  RPMs.rearLeft =
      (cos(radians(225.0)) * x) + (sin(radians(225.0)) * y) + rotation;
  RPMs.rearRight =
      (cos(radians(315.0)) * x) + (sin(radians(315.0)) * y) - rotation;

  return RPMs;
}

uint64_t combineMotorRPMs(Motor_RPMs RPMs) {
  uint64_t RPMdata;

  // 各モーターの RPM 値をシフトして結合（16ビットに制限）
  RPMdata |= ((uint64_t)((uint16_t)RPMs.frontLeft) << 48);
  RPMdata |= ((uint64_t)((uint16_t)RPMs.frontRight) << 32);
  RPMdata |= ((uint64_t)((uint16_t)RPMs.rearLeft) << 16);
  RPMdata |= (uint64_t)((uint16_t)RPMs.rearRight);

  return RPMdata;
}

void printControlData(const ControlData &data) {
  Serial.println("Control Data:");
  Serial.print("Left Stick X: ");
  Serial.println(data.left_x);
  Serial.print("Left Stick Y: ");
  Serial.println(data.left_y);

  Serial.print("Square: ");
  Serial.println(data.catching ? "Pressed" : "Not Pressed");
  Serial.print("Circle: ");
  Serial.println(data.r_launching ? "Pressed" : "Not Pressed");
  Serial.print("Triangle: ");
  Serial.println(data.l_launching ? "Pressed" : "Not Pressed");

  Serial.print("Left: ");
  Serial.println(data.left ? "Pressed" : "Not Pressed");
  Serial.print("Right: ");
  Serial.println(data.right ? "Pressed" : "Not Pressed");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  if (!CAN.begin(1000E3)) {
    Serial.println("ERROR:Starting CAN failed!");
    while (1)
      ;
    delay(1);
  }
  leftLaunchingServo.attach(LEFT_SERVO_PIN);
  rightLaunchingServo.attach(RIGHT_SERVO_PIN);
  leftLaunchingServo.write(left_launching_degree);
  rightLaunchingServo.write(right_launching_degree);
  
  pinMode(COLLECT_DIR,OUTPUT);
  ledcSetup(4,5000,8);
  ledcAttachPin(COLLECT_PWM,4);
}

void loop() {
  static String buffer = "";
  ControlData ps;
  Motor_RPMs RPMs;

  uint32_t receiveData = 0; // UART2の受信データ格納用
  uint64_t data = 0;        // モータのRPMデータ格納用

  uint8_t txData[8]; // CANの送信データ格納用

  while (Serial2.available()) {
    char incomingByte = Serial2.read();
    lastReceiveTime = millis();

    if (incomingByte == '\n') {
      if (buffer.length() >= sizeof(receiveData)) {
        memcpy(&receiveData, buffer.c_str(), sizeof(receiveData));
        memcpy(&ps, &receiveData,
               sizeof(ps)); // 32ビットの整数から構造体にコピー

        RPMs =
            calculateWheelRPMs(ps.left_x, ps.left_y, ps.left,
                               ps.right); // メカナムの各モーターのＲＰＭを計算
        data = combineMotorRPMs(RPMs); // 64bitのデータを取得

        printControlData(ps); // debug用

        for (int i = 0; i < 8; i++) {
          txData[i] =
              (data >> (56 - i * 8)) & 0xFF; // 64bitのデータを8bitに分割する
        }
        // CAN.beginPacket(ID);
        // CAN.write(txData, sizeof(txData));
        // CAN.endPacket();
      }
      buffer = ""; // buffer clear
    } else {
      buffer += incomingByte; // bufferに文字を追加
    }
  }

  // TODO serialが送られてきていない時の停止制御を書く
  if (millis() - lastReceiveTime > TIMEOUT &&
      buffer.length() >
          0) { // タイムアウト時間を超えた時かつ、 bufferが空でない時
    Serial.println("Buffer cleared");
    buffer = ""; // buffer clear
    data=0;
    for (int i = 0; i < 8; i++) {
          txData[i] =
              (data >> (56 - i * 8)) & 0xFF; // 64bitのデータを8bitに分割する
        }
    // CAN.beginPacket(ID);
    // CAN.write(txData, sizeof(txData));
    // CAN.endPacket();
  }
}