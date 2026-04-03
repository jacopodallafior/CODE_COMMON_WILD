#include <SPI.h>
#include <mcp2515.h>
#include <ArduPID.h>
#include "can_ids.h"          // ← added

MCP2515 mcp2515(10);
// Servo removed entirely

const int accelPin  = 3;
const int enablePin = 6;
const int latchPin  = 8;
const int clockPin  = 7;
const int dataPin   = 9;

const int minPWM = 38;
const int maxPWM = 225;

double P = 0.5, I = 1e-4, D = 1e-2;
ArduPID pidController;
double targetVelocity    = 0;
double actualGPSVelocity = 0;
double controlOutput     = 0;
double targetAcceleration = 0;
double actualAcceleration = 0;

int lastDisplayedNumber = -1;
bool lastBrakingState   = false;
byte standby            = B00001000;
const unsigned long displayInterval = 700;
unsigned long lastDisplayTime = millis();

uint32_t heartbeatInterval   = 200;
unsigned long nowMS           = millis();
unsigned long lastHbReceived  = millis();
unsigned long lastHbSent      = 0;
bool receivedHeartbeat        = false;

unsigned long lastCanSig      = millis();
const unsigned long canLEDDur = 50;

int mode = 0;

// #define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
  #define Debug Serial
#else
  #define Debug if (false) Serial
#endif

// ── Send brake % to stepper board ─────────────────────────────
void sendBrakePct(float pct) {
  pct = constrain(pct, 0.0, 100.0);
  struct can_frame brakeMsg;
  brakeMsg.can_id  = CAN_BRAKE_PCT;   // 0x130
  brakeMsg.can_dlc = 4;
  memcpy(brakeMsg.data, &pct, 4);
  mcp2515.sendMessage(&brakeMsg);
  Debug.print("Brake CAN sent: "); Debug.println(pct, 1);
}

void setup() {
  pinMode(accelPin,  OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(latchPin,  OUTPUT);
  pinMode(clockPin,  OUTPUT);
  pinMode(dataPin,   OUTPUT);

  digitalWrite(enablePin, HIGH);
  analogWrite(accelPin, minPWM);

  // Reset display
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, standby);
  shiftOut(dataPin, clockPin, LSBFIRST, standby);
  digitalWrite(latchPin, HIGH);

  Serial.begin(115200);
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  pidController.begin(&actualGPSVelocity, &controlOutput, &targetVelocity, P, I, D);
  pidController.setOutputLimits(-99, 99);
  pidController.setWindUpLimits(-1, 1);
  pidController.start();
}

void loop() {
  struct can_frame canMsg;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    if (canMsg.can_id == CAN_HB_STERFBOARD) {        // 0x01
      heartbeatInterval = (canMsg.data[0] << 8) | canMsg.data[1];
      mode              = canMsg.data[2];
      lastHbReceived    = millis();
      Debug.print("Mode: "); Debug.println(mode);
    }
    else if (canMsg.can_id == CAN_VELOCITY_TARGET) {  // 0x120
      float rv, ra;
      memcpy(&rv, canMsg.data,     4);
      memcpy(&ra, canMsg.data + 4, 4);
      targetVelocity     = rv;
      targetAcceleration = ra;
    }
    else if (canMsg.can_id == CAN_GPS_VELOCITY) {     // 0x121
      float rv, ra;
      memcpy(&rv, canMsg.data,     4);
      memcpy(&ra, canMsg.data, 4);
      actualGPSVelocity  = rv;
      actualAcceleration = ra;
    }
  }

  // ── Mode logic ───────────────────────────────────────────────
  if (!receivedHeartbeat || mode == 2) {
    // Emergency: cut throttle, full brake
    digitalWrite(enablePin, HIGH);
    analogWrite(accelPin, minPWM);
    sendBrakePct(100.0);
    controlOutput = -99;
  }
  else if (mode == 0) {
    // Manual: release everything
    digitalWrite(enablePin, LOW);
    sendBrakePct(0.0);
  }
  else {
    // Auto: run PID
    digitalWrite(enablePin, HIGH);

    if (targetVelocity <= 0) {
      analogWrite(accelPin, minPWM);
      sendBrakePct(50.0);
      displayNumber(50, true);
      controlOutput = -50;
    }
    else {
      pidController.compute();
      nowMS = millis();

      if (controlOutput >= 0) {
        // Accelerate
        int pwmValue = map(controlOutput, 0, 100, minPWM, maxPWM);
        analogWrite(accelPin, pwmValue);
        sendBrakePct(0.0);   // release brake
        if (nowMS - lastDisplayTime > displayInterval) {
          displayNumber(abs(controlOutput), false);
          lastDisplayTime = millis();
        }
      }
      else {
        // Brake — negative PID output → positive brake %
        float brakePct = (float)(-controlOutput);  // 0-99
        analogWrite(accelPin, minPWM);
        sendBrakePct(brakePct);
        if (nowMS - lastDisplayTime > displayInterval) {
          displayNumber(abs(controlOutput), true);
          lastDisplayTime = millis();
        }
      }
    }
  }

  // ── Heartbeat out ────────────────────────────────────────────
  byte data[8];
  float fo = (float)controlOutput;
  float fv = (float)actualGPSVelocity;
  memcpy(data,     &fo, 4);
  memcpy(data + 4, &fv, 4);
  receivedHeartbeat = heartbeat(CAN_HB_AXELBRAKE, data, sizeof(data)); // 0x12

  led_maintenance();
}

// ── Display, heartbeat, LED functions unchanged ────────────────
void displayNumber(int number, bool braking) {
  if (number == lastDisplayedNumber && braking == lastBrakingState) return;
  lastDisplayedNumber = number;
  lastBrakingState    = braking;
  byte digits[2]      = { number / 10, number % 10 };
  for (int i = 1; i >= 0; i--) {
    digitalWrite(latchPin, LOW);
    byte segVal = getSegmentValue(digits[i]);
    if (braking) segVal |= standby;
    shiftOut(dataPin, clockPin, LSBFIRST, segVal);
    digitalWrite(latchPin, HIGH);
  }
}

byte getSegmentValue(byte digit) {
  switch (digit) {
    case 0: return 0b01110111;
    case 1: return 0b00010100;
    case 2: return 0b10110011;
    case 3: return 0b10110110;
    case 4: return 0b11010100;
    case 5: return 0b11100110;
    case 6: return 0b11100111;
    case 7: return 0b00110100;
    case 8: return 0b11110111;
    case 9: return 0b11110110;
    default: return 0b00000000;
  }
}

bool heartbeat(uint32_t id, const byte* data, size_t dataSize) {
  unsigned long now = millis();
  if (dataSize > 8) return false;
  if (now - lastHbSent >= heartbeatInterval) {
    struct can_frame hb;
    hb.can_id  = id;
    hb.can_dlc = dataSize;
    memcpy(hb.data, data, dataSize);
    mcp2515.sendMessage(&hb);
    lastHbSent = now;
  }
  return (now - lastHbReceived < heartbeatInterval * 2);
}

void led_maintenance() {
  nowMS = millis();
}