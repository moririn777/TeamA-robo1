#include <Arduino.h>
#include <PS4Controller.h>
#include <CAN.h>

const int LENGTH = 1; //ロボットの中心からオムニまでの長さ
unsigned int ID = 0x555; //ID
unsigned long long data;
const uint8_t dead_zone = 30;//デッドゾーン

uint8_t TxData[8];

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
  if (!CAN.begin(1000E3)) {
    Serial.println("ERROR:Starting CAN failed!");
    CAN.beginPacket(ID);
    CAN.write(0, 1);  
    CAN.endPacket();  
    while (1);
    delay(1);
  }
  
  PS4.begin("00:00:00:00:00:00");
}

void loop() {
  
  if(!PS4.isConnected()){
    Serial.println("ERROR:Cant PS4Connect!!");
    return;
  }

  int8_t left_x = PS4.LStickX();
  int8_t left_y = PS4.LStickY();
  int8_t right_x = PS4.RStickX();
  //int8_t right_y = PS4.RStickY();

  Motor_RPMs RPMs;

  if(abs(left_x) < dead_zone && abs(left_y) < dead_zone && abs(right_x) <dead_zone){
    data = 0; //デッドゾーン以下の時データを0にする
  } else{
    RPMs = calculateWheelRPMs(left_x,left_y,right_x); //メカナムの各モーターのＲＰＭを計算
    data = combineMotorRPMs(RPMs); //64bitのデータを取得
  }

  Serial.printf("data: 0x%016llX\n", data);
  Serial.printf("FL::%x\r\n",uint16_t(RPMs.frontLeft));
  Serial.printf("FR::%X\r\n",uint16_t(RPMs.frontRight));
  Serial.printf("RL::%X\r\n",uint16_t(RPMs.rearLeft));
  Serial.printf("RR::%X\r\n",uint16_t(RPMs.rearRight));
  
  for (int i = 0; i < 8; i++) {
    TxData[i] = (data >> (56 - i * 8)) & 0xFF; //64bitのデータを8bitに分割する
  }

  CAN.beginPacket(ID);
  CAN.write(TxData, sizeof(TxData));  
  CAN.endPacket();  
  
  delay(10);
}