/*
 * UNO R3 + BMP73T104：接受樹莓派馬達命令（視覺跟隨）
 *
 * 接線（建議）：
 *   Pi UART TX  → UNO D10（SoftwareSerial RX）
 *   共地 GND    → GND
 *   不要接 5V 互相供電
 *
 * 協定（9600，一行一命令，大小寫皆可）：
 *   F  前進
 *   L  原地／弧線左轉（畫面左側有人）
 *   R  原地／弧線右轉（畫面右側有人）
 *   Z  自轉搜尋（找不到人）
 *   S  軟停
 *   X  緊急煞停
 *
 * 若約 CMD_TIMEOUT_MS 沒收到新命令 → 緊急煞停（安全）
 * USB 監看鮑率：115200
 */
#include <Wire.h>
#include <BMP73T104.h>
#include <SoftwareSerial.h>
#include <ctype.h>

SoftwareSerial piSerial(10, 11); // RX=D10 <- Pi TX；TX=D11 不接

const unsigned long CMD_TIMEOUT_MS = 1200;
const unsigned long CMD_REFRESH_MS = 250;

// 注意：BMP73T104 單路約 1.1A、總電流約 4A
// 原地左右輪對拉最容易堵轉觸發過流，之後會「整台假死」直到清故障／重上電
const int BASE_FORWARD_SPEED = 85;
const int BASE_TURN_SPEED = 60;   // 轉向勿太大
const int BASE_SPIN_SPEED = 50;   // 搜尋自轉更小
const int BASE_TURN_INNER = 20;   // 內輪慢轉（弧線轉，比正反轉對拉省電流）

// 規格書：I2C 指令間隔至少約 3ms；太短會出現有的輪轉有的不轉
const unsigned int MOTOR_CMD_GAP_US = 4000;

BMP73T104 myM1(2, DCMotor); // 左後
BMP73T104 myM2(2, DCMotor); // 右後
BMP73T104 myM3(2, DCMotor); // 右前
BMP73T104 myM4(2, DCMotor); // 左前

const float TRIM_M1 = 1.08;
const float TRIM_M2 = 1.08;
const float TRIM_M3 = 1.08;
const float TRIM_M4 = 1.08;

// 若某輪方向反了，改成 -1
const int DIR_M1 = 1;
const int DIR_M2 = 1;
const int DIR_M3 = 1;
const int DIR_M4 = 1;

enum Cmd : char {
  CMD_NONE = 0,
  CMD_FORWARD = 'F',
  CMD_LEFT = 'L',
  CMD_RIGHT = 'R',
  CMD_SPIN = 'Z',
  CMD_STOP = 'S',
  CMD_BRAKE = 'X',
};

char activeCmd = CMD_STOP;
unsigned long lastCmdTime = 0;
unsigned long lastApplyTime = 0;

void setSingleMotorSafe(BMP73T104 &motor, int rawSpeed, float trim, int dir) {
  int targetSpeed = constrain((int)(rawSpeed * trim * dir), -100, 100);
  motor.dcMotorRun(targetSpeed);
  delayMicroseconds(MOTOR_CMD_GAP_US);
}

void setLeftMotors(int speed) {
  setSingleMotorSafe(myM1, speed, TRIM_M1, DIR_M1);
  setSingleMotorSafe(myM4, speed, TRIM_M4, DIR_M4);
}

void setRightMotors(int speed) {
  setSingleMotorSafe(myM2, speed, TRIM_M2, DIR_M2);
  setSingleMotorSafe(myM3, speed, TRIM_M3, DIR_M3);
}

void applyDrive(int leftSpeed, int rightSpeed) {
  setLeftMotors(leftSpeed);
  setRightMotors(rightSpeed);
  lastApplyTime = millis();
}

void clearMotorFaults() {
  // 載重堵轉後可能卡在過流狀態；定期清一次
  myM1.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM2.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM3.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM4.clearOverCurrent();
  delayMicroseconds(MOTOR_CMD_GAP_US);
}

void emergencyBrake() {
  myM1.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM2.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM3.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  myM4.dcMotorBrake();
  delayMicroseconds(MOTOR_CMD_GAP_US);
  lastApplyTime = millis();
}

void softStop() {
  applyDrive(0, 0);
}

void applyCommand(char cmd, bool force) {
  if (!force && cmd == activeCmd && (millis() - lastApplyTime < CMD_REFRESH_MS)) {
    return;
  }

  // 從停止進入運動前先清過流，避免保護鎖死後怎麼送命令都不轉
  if (force && (activeCmd == CMD_STOP || activeCmd == CMD_BRAKE) &&
      (cmd == CMD_FORWARD || cmd == CMD_LEFT || cmd == CMD_RIGHT || cmd == CMD_SPIN)) {
    clearMotorFaults();
    Serial.println(F("cleared OCP before move"));
  }

  switch (cmd) {
    case CMD_FORWARD:
      applyDrive(BASE_FORWARD_SPEED, BASE_FORWARD_SPEED);
      break;
    case CMD_LEFT:
      // 弧線左轉（內輪慢、外輪快），比左右對拉較不易過流
      applyDrive(BASE_TURN_INNER, BASE_TURN_SPEED);
      break;
    case CMD_RIGHT:
      applyDrive(BASE_TURN_SPEED, BASE_TURN_INNER);
      break;
    case CMD_SPIN:
      // 搜尋才用慢速原地轉
      applyDrive(-BASE_SPIN_SPEED, BASE_SPIN_SPEED);
      break;
    case CMD_BRAKE:
      emergencyBrake();
      break;
    case CMD_STOP:
    default:
      softStop();
      cmd = CMD_STOP;
      break;
  }

  activeCmd = cmd;
}

bool isValidCmd(char c) {
  return c == CMD_FORWARD || c == CMD_LEFT || c == CMD_RIGHT ||
         c == CMD_SPIN || c == CMD_STOP || c == CMD_BRAKE;
}

void pollPiCommands() {
  static char buf[16];
  static uint8_t idx = 0;

  while (piSerial.available() > 0) {
    char c = piSerial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      idx = 0;

      // 接受 "F" / "f" / "FWD" 等形式，取第一個有效字母
      char cmd = 0;
      for (uint8_t i = 0; buf[i] != '\0'; i++) {
        char u = toupper(buf[i]);
        if (isValidCmd(u)) {
          cmd = u;
          break;
        }
      }
      if (cmd == 0) continue;

      lastCmdTime = millis();
      bool force = (cmd != activeCmd);
      applyCommand(cmd, force);

      Serial.print(F("CMD "));
      Serial.write(cmd);
      Serial.println();
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  piSerial.begin(9600);
  delay(500);

  Wire.begin();
  Wire.setClock(100000);
  delay(300);

  myM1.begin(M1); delay(100);
  myM2.begin(M2); delay(100);
  myM3.begin(M3); delay(100);
  myM4.begin(M4); delay(100);
  clearMotorFaults();

  emergencyBrake();
  activeCmd = CMD_STOP;
  lastCmdTime = millis();

  Serial.println(F("=== UNO Pi-slave 馬達控制就緒 ==="));
  Serial.println(F("命令: F/L/R/Z/S/X （Pi -> D10 @ 9600）"));
  Serial.println(F("供電建議: 18650->BMP73T104，短接 VIN jumper 給 UNO；勿用行充分開供電"));
}

void loop() {
  static unsigned long lastFaultClear = 0;

  pollPiCommands();

  // 逾時無命令 → 緊急煞停
  if (activeCmd != CMD_STOP && activeCmd != CMD_BRAKE &&
      (millis() - lastCmdTime > CMD_TIMEOUT_MS)) {
    Serial.println(F("Pi 命令逾時 -> 緊急煞停"));
    emergencyBrake();
    activeCmd = CMD_STOP;
  }

  // 前進／轉向時定期重送 PWM，避免驅動板掉指令
  if (activeCmd == CMD_FORWARD || activeCmd == CMD_LEFT ||
      activeCmd == CMD_RIGHT || activeCmd == CMD_SPIN) {
    applyCommand(activeCmd, false);
    // 約每秒清一次過流旗標，避免堵轉後整台「假死」
    if (millis() - lastFaultClear > 1000) {
      clearMotorFaults();
      lastFaultClear = millis();
    }
  }
}
