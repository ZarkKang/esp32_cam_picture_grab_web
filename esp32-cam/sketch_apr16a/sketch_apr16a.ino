// ==========  导入库  ==========
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <SPI.h>
#include "driver/ledc.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>

// ========== 引脚定义 ==========
// 超声波传感器
#define TRIG_PIN 36
#define ECHO_PIN 14

// 蜂鸣器
#define BUZZER_PIN 19
#define BUZZER_PWM_CHANNEL 0

// 天问模块
#define RX_PIN 38
#define TX_PIN 39
HardwareSerial VoiceSerial(1);

// 电机控制 (L298N)
#define MOTOR1_IN1 2
#define MOTOR1_IN2 1
#define MOTOR2_IN1 3
#define MOTOR2_IN2 4

// 传感器
#define SPECIAL_LIMIT_SWITCH 48
#define NORMAL_LIMIT_SWITCH 47
#define PHOTO_SENSOR 40

// 舵机
#define SERVO_PIN 20
#define SPEED 5  // 毫秒/度

// 旋转编码器
#define ENCODER_SW 9
#define ENCODER_DT 7
#define ENCODER_CLK 6

// OLED显示屏
#define OLED_SDA 35
#define OLED_SCL 13

// LED指示灯
#define LED1_PIN 42
#define LED2_PIN 41

// ========== 天问数据缓冲区 ==========
uint8_t receivedData = 0x00;
bool newDataReceived = false;

// ========== 参数定义 ==========
// 超声波传感器
#define DETECTION_DISTANCE 8

// 计数
#define TOTAL_SLOTS 12
int itemCount = 0;

// 光电传感器
const unsigned long SPECIAL_THRESHOLD = 300000;  // 300ms低电平阈值(微秒)
bool initialPositionFound = false;
int currentSlot = 1;
bool isCalibrated = false;
unsigned long oneRevolutionTime = 0;
unsigned long timePerSlot = 0;
bool calibrating = false;
int calibrationPhase = 0;

bool motor1HomeFound = false;

// 旋转编码器
int encoderValue = 0;
int lastEncoderCLK = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 20;

unsigned long lastButtonPress = 0;
volatile bool buttonPressed = false;

const unsigned long encoderDebounce = 2;
bool showingEncoderValue = false;
unsigned long lastEncoderInteraction = 0;
const unsigned long ENCODER_DISPLAY_TIMEOUT = 5000;

bool systemReady = true;
bool findsome = false;

// ========== WiFi配置 ==========
const char* ssid = "REDMI K Pad";
const char* password = "95595555";

// 静态IP配置
IPAddress local_IP(192, 168, 50, 60);
IPAddress gateway(192, 168, 50, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 50, 1);

// ========== 后端服务器配置（运行Flask的电脑） ==========
const char* backendIP = "192.168.50.100";  // 请改为电脑实际IP
const int backendPort = 5000;
const char* capturePath = "/capture";
const int requestTimeout = 5000;

// ========== HTTP 服务器 ==========
WebServer server(80);

// OLED对象
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

// ========== 函数声明 ==========
float getDistance();
void controlMotors(int motor, int speed, int dir);
void home();
void home1();
bool calibrateOneRevolution();
bool goToSlot(int targetSlot);
void storeItem();
void retrieveItem();
void retrieveItem2(int x);
void setAngle(int angle);
void smoothServoOperation();
void encoderTask();
void displayEncoderValue(int value);
void display1();
void display2();
void display3();
void display4();
void display5();
void display6();
void display7(int x);
void display8();
void display9();
void display10();
void display11();
void display13();
void display14();
void display15(int x);
void display17();
void display18(int x);
void displayCaptureSuccess();
void displayCaptureFailed();
void displayWiFiConnected();
void displayWiFiFailed();
void playSuccess();
void playError();
void playStartup();
void common();
void voice2();
void voice3();
void updateEEPROM();
void loadFromEEPROM();
bool connectWiFi();
bool checkWiFiConnection();
bool notifyBackendCapture();
void handleFetch();
void setupPins();
void setupOLED();

// ========== 超声波测距 ==========
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  
  return (distance > 0 && distance < 400) ? distance : 400;
}

// ========== 电机控制 ==========
void controlMotors(int motor, int speed, int dir) {
  if (motor == 1) {
    if (dir == 1) {
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
    } else if (dir == -1) {
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
    } else {
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
    }
  } else if (motor == 2) {
    if (dir == 1) {
      digitalWrite(MOTOR2_IN1, HIGH);
      digitalWrite(MOTOR2_IN2, LOW);
    } else if (dir == -1) {
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
    } else {
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);
    }
  }
}

// ========== 回原点（转盘电机） ==========
void home() {
  initialPositionFound = false;
  controlMotors(1, 255, -1);
  
  unsigned long startTime = millis();
  bool lastState = digitalRead(PHOTO_SENSOR);
  unsigned long lastTransitionTime = 0;
  
  while (!initialPositionFound && (millis() - startTime) < 30000) {
    int currentState = digitalRead(PHOTO_SENSOR);
    unsigned long currentTime = micros();
    
    if (currentState != lastState) {
      if (currentState == HIGH) {
        if (lastTransitionTime > 0) {
          unsigned long lowDuration = currentTime - lastTransitionTime;
          if (lowDuration > SPECIAL_THRESHOLD) {
            Serial.println("找到初始位置！");
            controlMotors(1, 0, 0);
            initialPositionFound = true;
            delay(2000);
          }
        }
      } else {
        lastTransitionTime = currentTime;
      }
      lastState = currentState;
    }
    delay(1);
  }
  
  if (!initialPositionFound) {
    controlMotors(1, 0, 0);
    Serial.println("寻找初始位置超时！");
  }
}

// ========== 层电机回原点 ==========
void home1() {
  motor1HomeFound = false;
  display10();
  
  Serial.println("开始寻找电机1初始位置...");
  controlMotors(2, 255, 1);
  
  unsigned long startTime = millis();
  unsigned long timeout = 10000;
  
  while (!motor1HomeFound && (millis() - startTime) < timeout) {
    int switchState = digitalRead(SPECIAL_LIMIT_SWITCH);
    if (switchState == HIGH) {
      motor1HomeFound = true;
      controlMotors(2, 0, 0);
      display11();
      Serial.println("电机1找到初始位置！");
      delay(2000);
      break;
    }
    delay(10);
  }
  
  if (!motor1HomeFound) {
    controlMotors(2, 0, 0);
    display8();
    Serial.println("电机1回原点超时！");
  }
  delay(1000);
}

// ========== 校准一圈时间 ==========
bool calibrateOneRevolution() {
  if (!initialPositionFound) {
    Serial.println("请先找到初始位置！");
    return false;
  }
  
  calibrationPhase = 1;
  calibrating = true;
  unsigned long firstGapTime = 0;
  int gapCount = 0;
  
  controlMotors(1, 255, -1);
  
  bool lastState = digitalRead(PHOTO_SENSOR);
  unsigned long lastTransitionTime = 0;
  
  while (calibrating && gapCount < 2) {
    int currentState = digitalRead(PHOTO_SENSOR);
    unsigned long currentTime = micros();
    
    if (currentState != lastState) {
      if (currentState == HIGH) {
        if (lastTransitionTime > 0) {
          unsigned long lowDuration = currentTime - lastTransitionTime;
          if (lowDuration > SPECIAL_THRESHOLD) {
            gapCount++;
            if (gapCount == 1) {
              firstGapTime = currentTime;
              calibrationPhase = 2;
              Serial.println("找到第一个特殊缺口，继续寻找第二个...");
            } else if (gapCount == 2) {
              unsigned long secondGapTime = currentTime;
              oneRevolutionTime = secondGapTime - firstGapTime;
              timePerSlot = oneRevolutionTime / 6;
              calibrating = false;
              controlMotors(1, 0, 0);
              Serial.println("校准完成！");
              isCalibrated = true;
              currentSlot = 1;
              return true;
            }
          }
        }
      } else {
        lastTransitionTime = currentTime;
      }
      lastState = currentState;
    }
    delay(1);
  }
  return false;
}

// ========== 转到指定槽位 ==========
bool goToSlot(int targetSlot) {
  int b = targetSlot / 6;
  int a = targetSlot % 6;
  
  if (a == currentSlot) {
    Serial.println("已经在目标位置");
    return true;
  }
  
  Serial.printf("从位置 %d 转到位置 %d\n", currentSlot, a);
  
  int slotDifference = (a > currentSlot) ? (a - currentSlot) : (6 - currentSlot + a);
  unsigned long moveTime = slotDifference * timePerSlot;
  
  controlMotors(1, 255, -1);
  unsigned long startTime = micros();
  unsigned long elapsedTime = 0;
  
  while (elapsedTime < moveTime) {
    elapsedTime = micros() - startTime;
    delayMicroseconds(100);
  }
  controlMotors(1, 0, 0);
  
  currentSlot = a;
  Serial.printf("已到达位置 %d\n", currentSlot);
  delay(1000);
  
  if (!findsome) {
    if (b == 1) {
      controlMotors(2, 255, -1);
      while (digitalRead(NORMAL_LIMIT_SWITCH) == LOW) { delay(1); }
      controlMotors(2, 0, 0);
      delay(1000);
    } else if (b == 0) {
      controlMotors(2, 255, 1);
      while (digitalRead(SPECIAL_LIMIT_SWITCH) == LOW) { delay(1); }
      controlMotors(2, 0, 0);
      delay(1000);
    }
  }
  return true;
}

// ========== 存货 ==========
void storeItem() {
  systemReady = false;
  display4();
  delay(1000);
  
  playSuccess();
  delay(500);
  
  display5();
  Serial.println("通知后端拍摄物品照片...");
  bool captureSuccess = notifyBackendCapture();
  
  if (captureSuccess) {
    displayCaptureSuccess();
  } else {
    displayCaptureFailed();
  }
  
  itemCount = (itemCount + 1) % TOTAL_SLOTS;
  if (itemCount == 0) itemCount = TOTAL_SLOTS;
  updateEEPROM();
  
  display7(itemCount);
  goToSlot(itemCount);
  smoothServoOperation();
  
  display17();
  playSuccess();
  
  systemReady = true;
}

// ========== 取物（编码器触发） ==========
void retrieveItem() {
  if (!systemReady) { display3(); return; }
  findsome = true;
  systemReady = false;
  
  display15(encoderValue);
  
  int mapping[13] = {0, 4, 5, 6, 1, 2, 3, 10, 11, 12, 7, 8, 9};
  if (encoderValue >= 1 && encoderValue <= 12) {
    goToSlot(mapping[encoderValue]);
  } else {
    goToSlot(1);
  }
  
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  voice2();
  
  if (encoderValue < 6) {
    display13();
    for (int k = 1; k < 5; k++) {
      digitalWrite(LED2_PIN, HIGH); delay(1000);
      digitalWrite(LED2_PIN, LOW); delay(1000);
    }
  } else {
    display14();
    for (int j = 1; j < 5; j++) {
      digitalWrite(LED1_PIN, HIGH); delay(1000);
      digitalWrite(LED1_PIN, LOW); delay(1000);
    }
  }
  
  playSuccess();
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  findsome = false;
  systemReady = true;
  showingEncoderValue = true;
  lastEncoderInteraction = millis();
}

// ========== 指定槽位取物（语音/HTTP指令） ==========
void retrieveItem2(int x) {
  if (!systemReady) { display3(); return; }
  findsome = true;
  systemReady = false;
  
  display15(x);
  
  int mapping[13] = {0, 4, 5, 6, 1, 2, 3, 10, 11, 12, 7, 8, 9};
  if (x >= 1 && x <= 12) {
    goToSlot(mapping[x]);
  } else {
    goToSlot(1);
  }
  
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  voice2();
  
  if (x < 8) {
    digitalWrite(LED1_PIN, HIGH);
    display13();
  } else {
    digitalWrite(LED2_PIN, HIGH);
    display14();
  }
  
  playSuccess();
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  findsome = false;
  systemReady = true;
  showingEncoderValue = true;
  lastEncoderInteraction = millis();
}

// ========== 舵机 ==========
void smoothServoOperation() {
  for (int angle = 0; angle <= 180; angle++) {
    setAngle(angle);
    delay(SPEED);
  }
  for (int angle = 180; angle >= 0; angle--) {
    setAngle(angle);
    delay(SPEED);
  }
}

void setAngle(int angle) {
  int pulse = map(angle, 0, 180, 500, 2500);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - pulse);
}

// ========== 旋转编码器 ==========
void encoderTask() {
  int currentCLK = digitalRead(ENCODER_CLK);
  int currentDT = digitalRead(ENCODER_DT);
  
  if (currentCLK != lastEncoderCLK) {
    if (millis() - lastDebounceTime > encoderDebounce) {
      if (currentCLK == LOW) {
        if (currentDT == LOW) {
          encoderValue++;
          if (encoderValue > TOTAL_SLOTS) encoderValue = 1;
        } else {
          encoderValue--;
          if (encoderValue < 1) encoderValue = TOTAL_SLOTS;
        }
        displayEncoderValue(encoderValue);
        showingEncoderValue = true;
        lastEncoderInteraction = millis();
        common();
      }
      lastDebounceTime = millis();
    }
  }
  lastEncoderCLK = currentCLK;
  
  int buttonReading = digitalRead(ENCODER_SW);
  if (buttonReading == LOW) {
    if ((millis() - lastButtonPress) > debounceDelay) {
      buttonPressed = true;
      lastButtonPress = millis();
      Serial.println("按钮被按下");
      display18(encoderValue);
      playSuccess();
      delay(100);
    }
  }
  
  if (buttonPressed) {
    buttonPressed = false;
    retrieveItem();
    delay(300);
  }
  lastEncoderInteraction = millis();
}

// ========== 天问模块 ==========
void voice2() {
  VoiceSerial.write(0xFF);
  Serial.println("发送取货成功信号: 0xFF");
  delay(10);
}

void voice3() {
  if (VoiceSerial.available()) {
    receivedData = VoiceSerial.read();
    newDataReceived = true;
  }
  if (newDataReceived) {
    Serial.printf("接收到语音命令: 0x%02X\n", receivedData);
    if (receivedData >= 0x00 && receivedData <= 0x0B) {
      retrieveItem2(receivedData + 1);
    } else {
      Serial.printf("警告：无效语音命令 0x%02X\n", receivedData);
    }
    newDataReceived = false;
  }
}

// ========== WiFi与后端通信 ==========
bool connectWiFi() {
  Serial.printf("正在连接WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi连接成功！IP: %s\n", WiFi.localIP().toString().c_str());
    displayWiFiConnected();
    return true;
  } else {
    Serial.println("WiFi连接失败");
    displayWiFiFailed();
    return false;
  }
}

bool checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi断开，尝试重连...");
    return connectWiFi();
  }
  return true;
}

bool notifyBackendCapture() {
  if (!checkWiFiConnection()) {
    Serial.println("WiFi未连接，无法通知后端");
    return false;
  }
  
  HTTPClient http;
  String url = "http://" + String(backendIP) + ":" + String(backendPort) + String(capturePath);
  Serial.printf("通知后端拍照: %s\n", url.c_str());
  
  http.begin(url);
  http.setTimeout(requestTimeout);
  
  int httpCode = http.GET();
  bool success = false;
  
  if (httpCode == HTTP_CODE_OK) {
    Serial.println("后端拍照成功");
    success = true;
  } else {
    Serial.printf("后端拍照失败，HTTP码: %d\n", httpCode);
  }
  http.end();
  return success;
}

// ========== HTTP 请求处理 ==========
void handleFetch() {
  if (server.hasArg("slot")) {
    int slot = server.arg("slot").toInt();
    if (slot >= 1 && slot <= 12) {
      Serial.printf("收到HTTP取物指令: 槽位 %d\n", slot);
      server.send(200, "text/plain", "OK");
      retrieveItem2(slot);
    } else {
      server.send(400, "text/plain", "Invalid slot");
    }
  } else {
    server.send(400, "text/plain", "Missing slot parameter");
  }
}

// ========== OLED显示函数 ==========
void displayEncoderValue(int value) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 5, "选择槽位:");
  u8g2.setFont(u8g2_font_logisoso24_tf);
  char numStr[4];
  sprintf(numStr, "%02d", value);
  int xPos = (128 - u8g2.getStrWidth(numStr)) / 2;
  u8g2.drawStr(xPos, 30, numStr);
  u8g2.sendBuffer();
}

void display1() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "系统空闲...");
  u8g2.sendBuffer();
}

void display2() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "检测到物品");
  u8g2.sendBuffer();
}

void display3() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "系统忙碌");
  u8g2.sendBuffer();
}

void display4() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "开始存货");
  u8g2.sendBuffer();
}

void display5() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "正在拍照...");
  u8g2.sendBuffer();
}

void display6() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "回原点中...");
  u8g2.sendBuffer();
}

void display7(int x) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 10, "移动到：");
  u8g2.setFont(u8g2_font_logisoso24_tf);
  char numStr[3];
  sprintf(numStr, "%02d", x);
  u8g2.drawStr(50, 30, numStr);
  u8g2.sendBuffer();
}

void display8() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "回原点失败");
  u8g2.sendBuffer();
}

void display9() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "回原点成功");
  u8g2.sendBuffer();
}

void display10() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "电机1回原点");
  u8g2.sendBuffer();
}

void display11() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "电机1回原点成功");
  u8g2.sendBuffer();
}

void display13() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "东西在第一层");
  u8g2.sendBuffer();
}

void display14() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "东西在第二层");
  u8g2.sendBuffer();
}

void display15(int x) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 10, "取出到：");
  u8g2.setFont(u8g2_font_logisoso24_tf);
  char numStr[3];
  sprintf(numStr, "%02d", x);
  u8g2.drawStr(50, 30, numStr);
  u8g2.sendBuffer();
}

void display17() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "存货完成");
  u8g2.sendBuffer();
}

void display18(int x) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 10, "您选择了：");
  u8g2.setFont(u8g2_font_logisoso24_tf);
  char numStr[3];
  sprintf(numStr, "%02d", x);
  u8g2.drawStr(50, 30, numStr);
  u8g2.sendBuffer();
}

void displayCaptureSuccess() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(20, 20, "拍照成功");
  u8g2.sendBuffer();
  delay(1000);
}

void displayCaptureFailed() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(20, 20, "拍照失败");
  u8g2.sendBuffer();
  delay(1000);
}

void displayWiFiConnected() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "WiFi已连接");
  u8g2.sendBuffer();
  delay(1000);
}

void displayWiFiFailed() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(10, 20, "WiFi连接失败");
  u8g2.drawUTF8(10, 40, "离线模式运行");
  u8g2.sendBuffer();
  delay(2000);
}

// ========== 蜂鸣器 ==========
void playSuccess() {
  ledcWriteTone(BUZZER_PWM_CHANNEL, 800);
  delay(150);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
  delay(50);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 1200);
  delay(300);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
}

void playError() {
  ledcWriteTone(BUZZER_PWM_CHANNEL, 300);
  delay(300);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
  delay(100);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 300);
  delay(300);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
}

void playStartup() {
  ledcWriteTone(BUZZER_PWM_CHANNEL, 800);
  delay(100);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
  delay(30);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 1200);
  delay(100);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
  delay(30);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 1600);
  delay(200);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
}

void common() {
  ledcWriteTone(BUZZER_PWM_CHANNEL, 1000);
  delay(10);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
}

// ========== EEPROM ==========
void updateEEPROM() {
  EEPROM.write(0, itemCount);
  EEPROM.commit();
}

void loadFromEEPROM() {
  itemCount = EEPROM.read(0);
  if (itemCount >= TOTAL_SLOTS) itemCount = 0;
}

// ========== 初始化引脚 ==========
void setupPins() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  
  pinMode(SPECIAL_LIMIT_SWITCH, INPUT_PULLDOWN);
  pinMode(NORMAL_LIMIT_SWITCH, INPUT_PULLDOWN);
  pinMode(PHOTO_SENSOR, INPUT_PULLUP);
  
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  
  pinMode(SERVO_PIN, OUTPUT);
  
  EEPROM.begin(512);
}

// ========== 初始化OLED ==========
void setupOLED() {
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(20, 30, "Initializing...");
  u8g2.sendBuffer();
  delay(1000);
}

// ========== 主程序 ==========
void setup() {
  Serial.begin(115200);
  Serial.println("系统启动中...");
  
  // PWM初始化（新版API）
  ledcAttach(BUZZER_PIN, 2000, 8);
  
  setupPins();
  setupOLED();
  loadFromEEPROM();
  
  // 配置静态IP
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("静态IP配置失败");
  }
  
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi连接成功！");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
    displayWiFiConnected();
  } else {
    Serial.println("\nWiFi连接失败");
    displayWiFiFailed();
  }
  
  // 启动HTTP服务器
  server.on("/fetch", handleFetch);
  server.begin();
  Serial.println("HTTP服务器已启动，监听端口80");
  
  VoiceSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  lastEncoderCLK = digitalRead(ENCODER_CLK);
  encoderValue = 1;
  displayEncoderValue(encoderValue);
  showingEncoderValue = true;
  lastEncoderInteraction = millis();
  
  display6();
  home();
  home1();
  calibrateOneRevolution();
  
  playStartup();
  Serial.println("系统初始化完成");
}

// ========== 主循环 ==========
void loop() {
  server.handleClient();  // 处理HTTP请求
  
  // 超声波检测存货
  static unsigned long lastDetection = 0;
  if (millis() - lastDetection > 500) {
    float distance = getDistance();
    if (distance < DETECTION_DISTANCE && systemReady) {
      showingEncoderValue = false;
      storeItem();
      lastDetection = millis() + 3000;
    } else {
      lastDetection = millis();
    }
  }
  
  // OLED显示管理
  if (showingEncoderValue) {
    if (millis() - lastEncoderInteraction > ENCODER_DISPLAY_TIMEOUT) {
      showingEncoderValue = false;
    }
  } else {
    display1();
  }
  
  encoderTask();
  voice3();
  
  delay(10);
}