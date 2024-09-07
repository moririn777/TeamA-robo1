#include <Arduino.h>
#include <CAN.h>
#include <ESP32Servo.h>

const int LENGTH = 1; //ロボットの中心からオムニまでの長さ
unsigned int ID = 0x555; //ID
unsigned long long data;
const uint8_t dead_zone = 30;//デッドゾーン

uint8_t TxData[8];

const uint8_t TXPIN = 2;
const uint8_t RXPIN = 0;

const uint8_t RIGHT_SERVO = 23;
const uint8_t LEFT_SERVO = 27;

const uint8_t COLLECT_PWM = 33;
const uint8_t TAKE_PIN = 32;


struct ControlData {
    int8_t left_x;    // 左スティックX
    int8_t left_y;    // 左スティックY
    int8_t right_x;   // 右スティックX
    int8_t right_y;   // 右スティックY

    uint8_t left_firing : 1; // 左発射
    uint8_t right_firing : 1; // 右発射
    uint8_t collect : 1;     // 回収
    uint8_t take : 1;        // 巻取

    uint8_t forward : 1;     // 前進
    uint8_t back : 1;        // 後退
    uint8_t left : 1;       // 左
    uint8_t right : 1;       // 右
};

struct Motor_RPMs{
  uint16_t frontLeft;
  uint16_t frontRight;
  uint16_t rearLeft;
  uint16_t rearRight;
};

Motor_RPMs calculateWheelRPMs(int x,int y, int rotation){ //オムニホイールの各RPMを代入
  Motor_RPMs RPMs;
  
  RPMs.frontLeft = (cos(radians(45.0)) * x) + (sin(radians(45.0)) * y) + rotation;
  RPMs.frontRight = (cos(radians(135.0)) * x) + (sin(radians(135.0)) * y) - rotation;
  RPMs.rearLeft = (cos(radians(225.0)) * x) + (sin(radians(225.0)) * y) + rotation;
  RPMs.rearRight = (cos(radians(315.0)) * x) + (sin(radians(315.0)) * y) - rotation;

  return RPMs;
}

unsigned long long combineMotorRPMs(Motor_RPMs RPMs) {
  unsigned long long  RPMdata;
  
  // 各モーターの RPM 値をシフトして結合（16ビットに制限）
  RPMdata |= ((unsigned long long)((uint16_t)RPMs.frontLeft) << 48);
  RPMdata |= ((unsigned long long)((uint16_t)RPMs.frontRight) << 32);
  RPMdata |= ((unsigned long long)((uint16_t)RPMs.rearLeft) << 16);
  RPMdata |= (unsigned long long)((uint16_t)RPMs.rearRight);

  return RPMdata;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  if (!Serial2) {
    Serial.println("ERROR: Serial2 initialization failed!");
    while (1); 
  }

  if (!CAN.begin(1000E3)) {
    Serial.println("ERROR:Starting CAN failed!");
    CAN.beginPacket(ID);
    CAN.write(0, 1);  
    CAN.endPacket();  
    while (1);
    delay(1);
  }
}

void loop() {
  ControlData ps; 
  Motor_RPMs RPMs;
  uint64_t receiveData;

  if (Serial2.available()) {
    // 構造体のサイズ分のデータをserial2から読み取る
    Serial.println("come data");
    receiveData = 0;
    Serial2.readBytes(reinterpret_cast<uint8_t*>(&receiveData), sizeof(receiveData));
    memcpy(&ps, &receiveData, sizeof(ps)); // 64ビットの整数から構造体にコピー

    RPMs = calculateWheelRPMs(ps.left_x,ps.left_y,ps.right_x); //メカナムの各モーターのＲＰＭを計算
    data = combineMotorRPMs(RPMs); //64bitのデータを取得

    Serial.printf("data: 0x%016llX\r\n", data);
    Serial.printf("FL::%d\r\n",(ps.left_x));
    Serial.printf("FR::%d\r\n",(ps.left_y));
    Serial.printf("RL::%d\r\n",(ps.right_x));
    Serial.printf("RR::%d\r\n",(ps.right_y));

    
    for (int i = 0; i < 8; i++) {
      TxData[i] = (data >> (56 - i * 8)) & 0xFF; //64bitのデータを8bitに分割する
    }

    //CAN.beginPacket(ID);
    //CAN.write(TxData, sizeof(TxData));  
    //CAN.endPacket();
    
    Serial2.flush();
  }
}