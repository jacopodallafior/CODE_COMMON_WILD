#include <SPI.h>
#include <mcp2515.h>
#include <ArduPID.h>
#include "can_ids.h"

MCP2515 mcp2515(10);

const int accelPin  = 3;
const int enablePin = 6;
const int latchPin  = 8;
const int clockPin  = 7;
const int dataPin   = 9;

const int minPWM = 38;
const int maxPWM = 225;

double P = 0.5, I = 1e-4, D = 1e-2;
ArduPID pidController;
double targetVelocity     = 0;
double actualGPSVelocity  = 0;
double controlOutput      = 0;
double targetAcceleration = 0;
double actualAcceleration = 0;

int lastDisplayedNumber = -1;
bool lastBrakingState   = false;
byte standby            = B00001000;
const unsigned long displayInterval = 700;
unsigned long lastDisplayTime = millis();

uint32_t heartbeatInterval   = 200;
unsigned long nowMS          = millis();
unsigned long lastHbReceived = millis();
unsigned long lastHbSent     = 0;
bool receivedHeartbeat       = false;

unsigned long lastCanSig     = millis();
const unsigned long canLEDDur = 50;

int mode = 0;

// NEW: brake CAN anti-spam
float lastBrakePctSent = -999.0;
unsigned long lastBrakeCanSent = 0;
const unsigned long brakeCanMinInterval = 50;   // 20 Hz
const float brakeCanMinDelta = 1.0;             // 1%

#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
  #define Debug Serial
#else
  #define Debug if (false) Serial
#endif

void sendBrakePct(float pct) {
  pct = constrain(pct, 0.0, 100.0);

  unsigned long now = millis();
  bool enoughTimePassed = (now - lastBrakeCanSent) >= brakeCanMinInterval;
  bool enoughChange = fabs(pct - lastBrakePctSent) >= brakeCanMinDelta;

  if (!enoughTimePassed && !enoughChange) {
    return;
  }

  struct can_frame brakeMsg;
  brakeMsg.can_id  = CAN_BRAKE_PCT;
  brakeMsg.can_dlc = 4;
  memcpy(brakeMsg.data, &pct, 4);
  mcp2515.sendMessage(&brakeMsg);

  lastBrakePctSent = pct;
  lastBrakeCanSent = now;

  Debug.print("Brake CAN sent: ");
  Debug.println(pct, 1);
}

void setup() {
  pinMode(accelPin,  OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(latchPin,  OUTPUT);
  pinMode(clockPin,  OUTPUT);
  pinMode(dataPin,   OUTPUT);

  digitalWrite(enablePin, HIGH);
  analogWrite(accelPin, minPWM);

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

    if (canMsg.can_id == CAN_HB_STERFBOARD) {
      heartbeatInterval = (canMsg.data[0] << 8) | canMsg.data[1];
      mode              = canMsg.data[2];
      lastHbReceived    = millis();
      Debug.print("Mode: ");
      Debug.println(mode);
    }
    else if (canMsg.can_id == CAN_VELOCITY_TARGET) {
      float rv, ra;
      memcpy(&rv, canMsg.data, 4);
      memcpy(&ra, canMsg.data + 4, 4);
      targetVelocity     = rv;
      targetAcceleration = ra;
    }
    else if (canMsg.can_id == CAN_GPS_VELOCITY) {
      float rv;
      memcpy(&rv, canMsg.data, 4);
      actualGPSVelocity  = rv;
      actualAcceleration = 0.0;
    }
  }

  if (!receivedHeartbeat || mode == 2) {
    digitalWrite(enablePin, HIGH);
    analogWrite(accelPin, minPWM);
    sendBrakePct(100.0);
    controlOutput = -99;
  }
  else if (mode == 0) {
    digitalWrite(enablePin, LOW);
    sendBrakePct(0.0);
  }
  else {
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
        int pwmValue = map(controlOutput, 0, 100, minPWM, maxPWM);
        analogWrite(accelPin, pwmValue);
        sendBrakePct(0.0);

        if (nowMS - lastDisplayTime > displayInterval) {
          displayNumber(abs(controlOutput), false);
          lastDisplayTime = millis();
        }
      }
      else {
        float brakePct = (float)(-controlOutput);
        analogWrite(accelPin, minPWM);
        sendBrakePct(brakePct);

        if (nowMS - lastDisplayTime > displayInterval) {
          displayNumber(abs(controlOutput), true);
          lastDisplayTime = millis();
        }
      }
    }
  }

  byte data[8];
  float fo = (float)controlOutput;
  float fv = (float)actualGPSVelocity;
  memcpy(data,     &fo, 4);
  memcpy(data + 4, &fv, 4);
  receivedHeartbeat = heartbeat(CAN_HB_AXELBRAKE, data, sizeof(data));

  led_maintenance();
}

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
