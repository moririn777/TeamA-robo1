#include <Arduino.h>
#include <CAN.h>
#include <ESP32Servo.h>

const int LENGTH = 1; //ロボットの中心からオムニまでの長さ
const unsigned int ID = 0x555; //ID

const uint8_t DEAD_ZONE = 30;//デッドゾーン

const unsigned long TIMEOUT = 50; //タイムアウト時間(ms)
static unsigned long lastReceiveTime = 0; //最後にデータが送られてきた時間


//TODO ピン番号を変更
const uint8_t TX_PIN = 2;
const uint8_t RX_PIN = 0;

const uint8_t RIGHT_SERVO = 23; //右発射用サーボ 
const uint8_t LEFT_SERVO = 27;  //左発射用サーボ

const uint8_t COLLECT_PWM = 33;  //回収用モータPWM
const uint8_t COLLECT_DIR = 0;   //DIR
const uint8_t TAKE_PWM = 32;     //巻取用モータPWM
const uint8_t TAKE_DIR = 0;      //DIR

const uint8_t LEFT_SENSOR = 0;   //左のセンサー
const uint8_t RIGHT_SENSOR = 0;  //右のセンサー

struct ControlData {
  int8_t left_x;    // 左スティックX
  int8_t left_y;    // 左スティックY
  int8_t right_x;   // 右スティックX
  int8_t right_y;   // 右スティックY

  uint8_t square : 1;   //四角ボタン
  uint8_t circle : 1;   //丸ボタン 
  uint8_t triangle : 1; //三角ボタン  
  uint8_t cross : 1;    //バツボタン  

  uint8_t up : 1;     // 上
  uint8_t down: 1;    // 下
  uint8_t left : 1;   // 左
  uint8_t right : 1;  // 右
};

struct Motor_RPMs{
  uint16_t frontLeft;
  uint16_t frontRight;
  uint16_t rearLeft;
  uint16_t rearRight;
};

Motor_RPMs calculateWheelRPMs(int x,int y, int rotation){ //オムニホイールの各RPMを代入
  Motor_RPMs rpms;
  
  rpms.frontLeft = (cos(radians(45.0)) * x) + (sin(radians(45.0)) * y) + rotation;
  rpms.frontRight = (cos(radians(135.0)) * x) + (sin(radians(135.0)) * y) - rotation;
  rpms.rearLeft = (cos(radians(225.0)) * x) + (sin(radians(225.0)) * y) + rotation;
  rpms.rearRight = (cos(radians(315.0)) * x) + (sin(radians(315.0)) * y) - rotation;

  return rpms;
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

void printControlData(const ControlData& data) {
  Serial.println("Control Data:");
  Serial.print("Left Stick X: "); Serial.println(data.left_x);
  Serial.print("Left Stick Y: "); Serial.println(data.left_y);
  Serial.print("Right Stick X: "); Serial.println(data.right_x);
  Serial.print("Right Stick Y: "); Serial.println(data.right_y);

  Serial.print("Square: "); Serial.println(data.square ? "Pressed" : "Not Pressed");
  Serial.print("Circle: "); Serial.println(data.circle ? "Pressed" : "Not Pressed");
  Serial.print("Triangle: "); Serial.println(data.triangle ? "Pressed" : "Not Pressed");
  Serial.print("Cross: "); Serial.println(data.cross ? "Pressed" : "Not Pressed");

  Serial.print("Up: "); Serial.println(data.up ? "Pressed" : "Not Pressed");
  Serial.print("Down: "); Serial.println(data.down ? "Pressed" : "Not Pressed");
  Serial.print("Left: "); Serial.println(data.left ? "Pressed" : "Not Pressed");
  Serial.print("Right: "); Serial.println(data.right ? "Pressed" : "Not Pressed");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

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
  static String buffer ="";
  ControlData ps; 
  Motor_RPMs rpms;

  uint64_t receiveData=0;  //UART2の受信データ格納用
  unsigned long long data; //モータのRPMデータ格納用
  
  uint8_t txData[8];  //CANの送信データ格納用
  
  while (Serial2.available()) {
    char incomingByte = Serial2.read();
    lastReceiveTime = millis();

    if(incomingByte == '\n'){
      if(buffer.length() >= sizeof(receiveData)){
        memcpy(&receiveData,buffer.c_str(),sizeof(receiveData));
        memcpy(&ps, &receiveData, sizeof(ps)); // 64ビットの整数から構造体にコピー

        rpms = calculateWheelRPMs(ps.left_x,ps.left_y,ps.right_x); //メカナムの各モーターのＲＰＭを計算
        data = combineMotorRPMs(rpms); //64bitのデータを取得

        printControlData(ps); //debug用

        for (int i = 0; i < 8; i++) {
          txData[i] = (data >> (56 - i * 8)) & 0xFF; //64bitのデータを8bitに分割する
        }
        //CAN.beginPacket(ID);
        //CAN.write(txData, sizeof(txData));  
        //CAN.endPacket();
      }     
      buffer = "";  //buffer clear
    }else{
      buffer += incomingByte; //bufferに文字を追加
    }
  }
  //TODO serialが送られてきていない時の停止制御を書く
  if(millis()-lastReceiveTime > TIMEOUT && buffer.length() > 0){ //タイムアウト時間を超えた時かつ、 bufferが空でない時
    Serial.println("Buffer cleared");
    buffer = "";  //buffer clear
  }
}