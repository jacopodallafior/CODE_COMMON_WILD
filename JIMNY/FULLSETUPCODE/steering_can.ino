/*
 * steering_can.ino
 *
 * Versione modificata:
 * encoder su D4 e D5 invece di A3 e A1
 */

#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <EnableInterrupt.h>
#include <SPI.h>
#include <mcp2515.h>
#include "can_ids.h"

// =========================
// ENCODER
// =========================
const int clkPin = 4;   // D4 = CHA
const int dtPin  = 5;   // D5 = CHB

volatile long counter = 0;
volatile uint8_t lastState = 0;
volatile unsigned long isrEvents = 0;
volatile unsigned long invalidTransitions = 0;

int encoderCountsPerRev = 4000;
double sprocketGearRatio = 2.5;
bool encoderCwRight = false;

// =========================
// DAC
// =========================
Adafruit_MCP4728 mcp;

const float DAC_VDD     = 4.98;
const int   DAC_MAX     = 4095;
const float IDLE_A      = 2.5200;
const float IDLE_B      = 2.4900;
const float DELTA_LIMIT = 0.80;

float CAL_A = 0.0000;
float CAL_B = 0.0000;

float    lastOutA  = IDLE_A;
float    lastOutB  = IDLE_B;
uint16_t lastCodeA = 0;
uint16_t lastCodeB = 0;

// =========================
// PID
// =========================
float targetAngleDeg   = 0.0;
float measuredAngleDeg = 0.0;

float Kp = 0.020;
float Ki = 0.012;
float Kd = 0.000;

float integralTerm  = 0.0;
const float integralLimit = 500.0;

float prevAngleDeg = 0.0;
float deltaCmd     = 0.0;

float lastErrorDeg = 0.0;
float lastPTerm    = 0.0;
float lastITerm    = 0.0;
float lastDTerm    = 0.0;

bool pidEnabled = false;

// =========================
// TIMING
// =========================
const unsigned long controlPeriodMs = 10;
unsigned long lastControlMs = 0;

const unsigned long printIntervalMs = 50;
unsigned long lastPrintMs = 0;

// =========================
// CAN
// =========================
MCP2515 mcp2515(10);
struct can_frame canMsg;

int      mode               = 0;
uint32_t heartbeatInterval  = 200;
unsigned long lastHbReceived = 0;
unsigned long lastHbSent     = 0;

unsigned long lastCanSig      = 0;
const unsigned long CAN_LED_DUR = 50;

// #define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
  #define Debug Serial
#else
  #define Debug if(false) Serial
#endif

// =========================
// HELPERS
// =========================
double getSteeringAngleDeg(long countsSnapshot) {
  double revs  = (double)countsSnapshot / (double)encoderCountsPerRev;
  double angle = 360.0 * revs / sprocketGearRatio;
  return angle * (encoderCwRight ? -1.0 : 1.0);
}

uint16_t voltToCode(float v) {
  if (v < 0.0) v = 0.0;
  if (v > DAC_VDD) v = DAC_VDD;
  return (uint16_t)((v / DAC_VDD) * DAC_MAX + 0.5);
}

void applyDelta(float delta) {
  if (delta >  DELTA_LIMIT) delta =  DELTA_LIMIT;
  if (delta < -DELTA_LIMIT) delta = -DELTA_LIMIT;

  deltaCmd = delta;

  float outA = IDLE_A + deltaCmd + CAL_A;
  float outB = IDLE_B - deltaCmd + CAL_B;

  outA = constrain(outA, 0.0, DAC_VDD);
  outB = constrain(outB, 0.0, DAC_VDD);

  lastCodeA = voltToCode(outA);
  lastCodeB = voltToCode(outB);

  mcp.setChannelValue(MCP4728_CHANNEL_A, lastCodeA);
  mcp.setChannelValue(MCP4728_CHANNEL_B, lastCodeB);

  lastOutA = outA;
  lastOutB = outB;
}

void resetPID() {
  integralTerm = 0.0;
  lastErrorDeg = 0.0;
  lastPTerm    = 0.0;
  lastITerm    = 0.0;
  lastDTerm    = 0.0;
  prevAngleDeg = measuredAngleDeg;
}

void stopOutput() {
  resetPID();
  applyDelta(0.0);
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin,  INPUT_PULLUP);

  lastState = (digitalRead(clkPin) << 1) | digitalRead(dtPin);

  enableInterrupt(clkPin, handleEncoder, CHANGE);
  enableInterrupt(dtPin,  handleEncoder, CHANGE);

  Wire.begin();
  Wire.setClock(400000);
  if (!mcp.begin()) {
    Serial.println("ERR,MCP4728_NOT_FOUND");
    while (1) {}
  }

  measuredAngleDeg = getSteeringAngleDeg(0);
  prevAngleDeg     = measuredAngleDeg;
  applyDelta(0.0);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  lastHbReceived = millis();
  lastHbSent     = millis();

  Serial.println("READY");
  Serial.println("DATA,t_ms,target_deg,angle_deg,error_deg,delta_v,va_out,vb_out,codeA,codeB,count,pid_on,p_term,i_term,d_term,isr,invalid");
  Serial.println("Serial cmds: e/d=PID on/off | z=zero encoder | r=reverse | s<deg>=set target | kp/ki/kd<val>=tune");
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  unsigned long now = millis();

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == CAN_HB_STERFBOARD) {
      heartbeatInterval = (canMsg.data[0] << 8) | canMsg.data[1];
      mode              = canMsg.data[2];
      lastHbReceived    = now;
      Debug.print("Mode: "); Debug.println(mode);

      if (mode == 1 && !pidEnabled) {
        resetPID();
        pidEnabled = true;
        Serial.println("INFO,CAN_AUTO_PID_ENABLED");
      }
      else if (mode != 1 && pidEnabled) {
        pidEnabled = false;
        stopOutput();
        Serial.println("INFO,CAN_PID_DISABLED");
      }
    }
    else if (canMsg.can_id == CAN_STEERING_TARGET) {
      float rxAngle, rxAngleVel;
      memcpy(&rxAngle,    canMsg.data,     4);
      memcpy(&rxAngleVel, canMsg.data + 4, 4);

      if (mode == 1) {
        if (abs(rxAngle - targetAngleDeg) > 5.0) integralTerm = 0.0;
        targetAngleDeg = rxAngle;
        Debug.print("Target: "); Debug.println(targetAngleDeg, 2);
      }
    }
  }

  if (now - lastHbReceived > heartbeatInterval * 2) {
    if (pidEnabled) {
      pidEnabled = false;
      stopOutput();
      Serial.println("WARN,WATCHDOG_NO_HB");
    }
  }

  if (now - lastHbSent >= heartbeatInterval) {
    struct can_frame hb;
    hb.can_id  = CAN_HB_STEERING;
    hb.can_dlc = 8;
    float fDelta = deltaCmd;
    float fAngle = measuredAngleDeg;
    memcpy(hb.data,     &fDelta, 4);
    memcpy(hb.data + 4, &fAngle, 4);
    mcp2515.sendMessage(&hb);
    lastHbSent = now;
  }

  if (now - lastControlMs >= controlPeriodMs) {
    lastControlMs = now;
    runControl();
  }

  if (now - lastPrintMs >= printIntervalMs) {
    lastPrintMs = now;
    printDataLine();
  }

  handleSerial();
}

// =========================
// PID CONTROL
// =========================
void runControl() {
  long countsSnapshot;
  noInterrupts();
  countsSnapshot = counter;
  interrupts();

  measuredAngleDeg = getSteeringAngleDeg(countsSnapshot);

  if (!pidEnabled) {
    applyDelta(0.0);
    prevAngleDeg = measuredAngleDeg;
    lastErrorDeg = targetAngleDeg - measuredAngleDeg;
    lastPTerm = lastITerm = lastDTerm = 0.0;
    return;
  }

  float dt    = controlPeriodMs / 1000.0f;
  float error = targetAngleDeg - measuredAngleDeg;
  float dMeas = (measuredAngleDeg - prevAngleDeg) / dt;

  integralTerm += error * dt;
  integralTerm  = constrain(integralTerm, -integralLimit, integralLimit);

  lastPTerm = Kp * error;
  lastITerm = Ki * integralTerm;
  lastDTerm = -Kd * dMeas;

  float u = constrain(lastPTerm + lastITerm + lastDTerm, -DELTA_LIMIT, DELTA_LIMIT);

  applyDelta(u);

  lastErrorDeg = error;
  prevAngleDeg = measuredAngleDeg;
}

// =========================
// ENCODER ISR
// =========================
void handleEncoder() {
  uint8_t clkState = digitalRead(clkPin);
  uint8_t dtState  = digitalRead(dtPin);
  uint8_t newState = (clkState << 1) | dtState;

  switch ((lastState << 2) | newState) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000:
      counter++; isrEvents++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100:
      counter--; isrEvents++; break;
    default:
      if (newState != lastState) invalidTransitions++;
      break;
  }
  lastState = newState;
}

// =========================
// SERIAL PRINT
// =========================
void printDataLine() {
  long c; unsigned long isr, inv;
  noInterrupts(); c = counter; isr = isrEvents; inv = invalidTransitions; interrupts();

  Serial.print("DATA,");  Serial.print(millis());
  Serial.print(",");      Serial.print(targetAngleDeg, 3);
  Serial.print(",");      Serial.print(measuredAngleDeg, 3);
  Serial.print(",");      Serial.print(lastErrorDeg, 3);
  Serial.print(",");      Serial.print(deltaCmd, 4);
  Serial.print(",");      Serial.print(lastOutA, 4);
  Serial.print(",");      Serial.print(lastOutB, 4);
  Serial.print(",");      Serial.print(lastCodeA);
  Serial.print(",");      Serial.print(lastCodeB);
  Serial.print(",");      Serial.print(c);
  Serial.print(",");      Serial.print(pidEnabled ? 1 : 0);
  Serial.print(",");      Serial.print(lastPTerm, 6);
  Serial.print(",");      Serial.print(lastITerm, 6);
  Serial.print(",");      Serial.print(lastDTerm, 6);
  Serial.print(",");      Serial.print(isr);
  Serial.print(",");      Serial.println(inv);
}

// =========================
// SERIAL COMMANDS
// =========================
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s == "e" || s == "E") { pidEnabled = true;  resetPID(); Serial.println("INFO,PID_ENABLED");  return; }
  if (s == "d" || s == "D") { pidEnabled = false; stopOutput(); Serial.println("INFO,PID_DISABLED"); return; }
  if (s == "z" || s == "Z") {
    noInterrupts(); counter = 0; invalidTransitions = 0; isrEvents = 0; interrupts();
    measuredAngleDeg = prevAngleDeg = 0.0;
    resetPID();
    Serial.println("INFO,ENCODER_ZEROED");
    return;
  }
  if (s == "p" || s == "P") { printDataLine(); return; }
  if (s == "r" || s == "R") {
    encoderCwRight = !encoderCwRight;
    Serial.print("INFO,ENCODER_REVERSED,"); Serial.println(encoderCwRight ? 1 : 0);
    return;
  }
  if (s.startsWith("s")) {
    float t = s.substring(1).toFloat();
    if (abs(t - targetAngleDeg) > 5.0) integralTerm = 0.0;
    targetAngleDeg = t;
    Serial.print("INFO,TARGET_DEG,"); Serial.println(targetAngleDeg, 3);
    return;
  }
  if (s.startsWith("kp")) { Kp = s.substring(2).toFloat(); Serial.print("INFO,KP,"); Serial.println(Kp, 6); return; }
  if (s.startsWith("ki")) { Ki = s.substring(2).toFloat(); Serial.print("INFO,KI,"); Serial.println(Ki, 6); return; }
  if (s.startsWith("kd")) { Kd = s.substring(2).toFloat(); Serial.print("INFO,KD,"); Serial.println(Kd, 6); return; }
}
