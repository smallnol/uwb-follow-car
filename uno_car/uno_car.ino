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

const unsigned long CMD_TIMEOUT_MS = 800;
const unsigned long CMD_REFRESH_MS = 200;

// 載重較重時可再往上調（上限約 100）
const int BASE_FORWARD_SPEED = 95;
const int BASE_TURN_SPEED = 85;
const int BASE_SPIN_SPEED = 80;

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

void applyDrive(int leftSpeed, int rightSpeed) {
  setLeftMotors(leftSpeed);
  setRightMotors(rightSpeed);
  lastApplyTime = millis();
}

void emergencyBrake() {
  myM1.dcMotorBrake();
  delayMicroseconds(800);
  myM2.dcMotorBrake();
  delayMicroseconds(800);
  myM3.dcMotorBrake();
  delayMicroseconds(800);
  myM4.dcMotorBrake();
  lastApplyTime = millis();
}

void softStop() {
  applyDrive(0, 0);
}

void applyCommand(char cmd, bool force) {
  if (!force && cmd == activeCmd && (millis() - lastApplyTime < CMD_REFRESH_MS)) {
    return;
  }

  switch (cmd) {
    case CMD_FORWARD:
      applyDrive(BASE_FORWARD_SPEED, BASE_FORWARD_SPEED);
      break;
    case CMD_LEFT:
      // 原地左轉：左退右進，讓鏡頭把左側的人轉回畫面中央
      applyDrive(-BASE_TURN_SPEED, BASE_TURN_SPEED);
      break;
    case CMD_RIGHT:
      applyDrive(BASE_TURN_SPEED, -BASE_TURN_SPEED);
      break;
    case CMD_SPIN:
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

  emergencyBrake();
  activeCmd = CMD_STOP;
  lastCmdTime = millis();

  Serial.println(F("=== UNO Pi-slave 馬達控制就緒 ==="));
  Serial.println(F("命令: F/L/R/Z/S/X （Pi -> D10 @ 9600）"));
}

void loop() {
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
  }
}
