/*
 * UNO R3 + BMP73T104：UWB 直線跟隨
 * 接收：D10 <- 車上 ESP32 GPIO17（9600，純數字距離 + 換行）
 * USB 除錯：115200
 */
#include <Wire.h>
#include <BMP73T104.h>
#include <SoftwareSerial.h>
#include <stdlib.h>

SoftwareSerial uwbSerial(10, 11); // RX=D10, TX=D11 不要接到 ESP32

const float TARGET_START_DIST = 2.0;
const float TARGET_STOP_DIST  = 1.8;
const unsigned long TIMEOUT_MS = 1000;
const unsigned long CMD_REFRESH_MS = 250;

float currentDistance = 0.0;
unsigned long lastReceiveTime = 0;
unsigned long lastCmdTime = 0;
bool isMoving = false;
bool hasDistance = false;

BMP73T104 myM1(2, DCMotor); // 左後
BMP73T104 myM2(2, DCMotor); // 右後
BMP73T104 myM3(2, DCMotor); // 右前
BMP73T104 myM4(2, DCMotor); // 左前

const int BASE_STRAIGHT_SPEED = 75;

const float TRIM_M1 = 1.08;
const float TRIM_M2 = 1.08;
const float TRIM_M3 = 1.08;
const float TRIM_M4 = 1.08;

// 若某輪反轉，改成 -1
const int DIR_M1 = 1;
const int DIR_M2 = 1;
const int DIR_M3 = 1;
const int DIR_M4 = 1;

void setSingleMotorSafe(BMP73T104 &motor, int rawSpeed, float trim, int dir) {
  int targetSpeed = constrain((int)(rawSpeed * trim * dir), -100, 100);
  motor.dcMotorRun(targetSpeed);
  delayMicroseconds(800);
}

void setLeftMotors(int speed) {
  setSingleMotorSafe(myM1, speed, TRIM_M1, DIR_M1);
  setSingleMotorSafe(myM4, speed, TRIM_M4, DIR_M4);
}

void setRightMotors(int speed) {
  setSingleMotorSafe(myM2, speed, TRIM_M2, DIR_M2);
  setSingleMotorSafe(myM3, speed, TRIM_M3, DIR_M3);
}

void applyDrive(int speed) {
  setLeftMotors(speed);
  setRightMotors(speed);
  lastCmdTime = millis();
}

void moveForward(bool force) {
  if (force || !isMoving || (millis() - lastCmdTime >= CMD_REFRESH_MS)) {
    applyDrive(BASE_STRAIGHT_SPEED);
    isMoving = true;
  }
}

void stopMotors(bool emergency) {
  if (emergency) {
    myM1.dcMotorBrake();
    delayMicroseconds(800);
    myM2.dcMotorBrake();
    delayMicroseconds(800);
    myM3.dcMotorBrake();
    delayMicroseconds(800);
    myM4.dcMotorBrake();
  } else {
    applyDrive(0);
  }
  isMoving = false;
}

void handleDistance(float dist) {
  currentDistance = dist;
  lastReceiveTime = millis();
  hasDistance = true;

  Serial.print(F("目前距離: "));
  Serial.print(currentDistance);
  Serial.print(F(" m -> "));

  if (currentDistance >= TARGET_START_DIST) {
    Serial.println(F("[前進跟隨]"));
    moveForward(false);
  } else if (currentDistance < TARGET_STOP_DIST) {
    Serial.println(F("[到達安全距離 -> 停止]"));
    if (isMoving) stopMotors(false);
  } else {
    Serial.println(isMoving ? F("[緩衝區 -> 維持前進]") : F("[緩衝區 -> 維持停止]"));
    if (isMoving) moveForward(false);
  }
}

void pollUwb() {
  static char buf[24];
  static uint8_t idx = 0;

  while (uwbSerial.available() > 0) {
    char c = uwbSerial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;

      float parsedDist = atof(buf);
      if (parsedDist > 0.05f && parsedDist < 30.0f) {
        handleDistance(parsedDist);
      }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  uwbSerial.begin(9600);
  delay(500);

  Wire.begin();
  Wire.setClock(100000);
  delay(300);

  myM1.begin(M1); delay(100);
  myM2.begin(M2); delay(100);
  myM3.begin(M3); delay(100);
  myM4.begin(M4); delay(100);

  stopMotors(true);

  Serial.println(F("=== UWB 直線跟隨測試啟動 (門檻: >= 2.0m 前進) ==="));
}

void loop() {
  pollUwb();

  if (isMoving && hasDistance && (millis() - lastReceiveTime > TIMEOUT_MS)) {
    Serial.println(F("UWB 訊號超時中斷 -> 緊急煞停"));
    stopMotors(true);
    hasDistance = false;
  }
}
