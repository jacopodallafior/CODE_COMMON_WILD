#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <EnableInterrupt.h>

// =========================
// ENCODER
// =========================
const int clkPin = A3;
const int dtPin  = A1;

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

const float DAC_VDD = 4.98;
const int DAC_MAX = 4095;

const float IDLE_A = 2.5200;
const float IDLE_B = 2.4900;

const float DELTA_LIMIT = 0.80;

float CAL_A = 0.0000;
float CAL_B = 0.0000;

float lastOutA = IDLE_A;
float lastOutB = IDLE_B;
uint16_t lastCodeA = 0;
uint16_t lastCodeB = 0;

// =========================
// PID
// =========================
float targetAngleDeg = 0.0;
float measuredAngleDeg = 0.0;

float Kp = 0.020;
float Ki = 0.012;
float Kd = 0.000;

float integralTerm = 0.0;
const float integralLimit = 500.0;

float prevAngleDeg = 0.0;
float deltaCmd = 0.0;

float lastErrorDeg = 0.0;
float lastPTerm = 0.0;
float lastITerm = 0.0;
float lastDTerm = 0.0;

bool pidEnabled = false;

// =========================
// TIMING
// =========================
const unsigned long controlPeriodMs = 10;
unsigned long lastControlMs = 0;

const unsigned long printIntervalMs = 50;
unsigned long lastPrintMs = 0;

// =========================
// HELPERS
// =========================
void handleEncoder();
void runControl();
void handleSerial();
void printDataLine();

double getEncoderRevolutions(long countsSnapshot) {
  return (double)countsSnapshot / (double)encoderCountsPerRev;
}

double getSteeringAngleDeg(long countsSnapshot) {
  double encoderRevs = getEncoderRevolutions(countsSnapshot);
  double angle = 360.0 * encoderRevs / sprocketGearRatio;
  angle *= encoderCwRight ? -1.0 : 1.0;
  return angle;
}

uint16_t voltToCode(float v) {
  if (v < 0.0) v = 0.0;
  if (v > DAC_VDD) v = DAC_VDD;
  return (uint16_t)((v / DAC_VDD) * DAC_MAX + 0.5);
}

void applyDelta(float delta) {
  if (delta > DELTA_LIMIT) delta = DELTA_LIMIT;
  if (delta < -DELTA_LIMIT) delta = -DELTA_LIMIT;

  deltaCmd = delta;

  float outA = IDLE_A + deltaCmd + CAL_A;
  float outB = IDLE_B - deltaCmd + CAL_B;

  if (outA < 0.0) outA = 0.0;
  if (outA > DAC_VDD) outA = DAC_VDD;

  if (outB < 0.0) outB = 0.0;
  if (outB > DAC_VDD) outB = DAC_VDD;

  uint16_t codeA = voltToCode(outA);
  uint16_t codeB = voltToCode(outB);

  mcp.setChannelValue(MCP4728_CHANNEL_A, codeA);
  mcp.setChannelValue(MCP4728_CHANNEL_B, codeB);

  lastOutA = outA;
  lastOutB = outB;
  lastCodeA = codeA;
  lastCodeB = codeB;
}

void resetPID() {
  integralTerm = 0.0;
  lastErrorDeg = 0.0;
  lastPTerm = 0.0;
  lastITerm = 0.0;
  lastDTerm = 0.0;
  prevAngleDeg = measuredAngleDeg;
}

void stopOutput() {
  resetPID();
  applyDelta(0.0);
}

void printDataLine() {
  long countsSnapshot;
  unsigned long isrSnapshot;
  unsigned long invalidSnapshot;

  noInterrupts();
  countsSnapshot = counter;
  isrSnapshot = isrEvents;
  invalidSnapshot = invalidTransitions;
  interrupts();

  Serial.print("DATA,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(targetAngleDeg, 3);
  Serial.print(",");
  Serial.print(measuredAngleDeg, 3);
  Serial.print(",");
  Serial.print(lastErrorDeg, 3);
  Serial.print(",");
  Serial.print(deltaCmd, 4);
  Serial.print(",");
  Serial.print(lastOutA, 4);
  Serial.print(",");
  Serial.print(lastOutB, 4);
  Serial.print(",");
  Serial.print(lastCodeA);
  Serial.print(",");
  Serial.print(lastCodeB);
  Serial.print(",");
  Serial.print(countsSnapshot);
  Serial.print(",");
  Serial.print(pidEnabled ? 1 : 0);
  Serial.print(",");
  Serial.print(lastPTerm, 6);
  Serial.print(",");
  Serial.print(lastITerm, 6);
  Serial.print(",");
  Serial.print(lastDTerm, 6);
  Serial.print(",");
  Serial.print(isrSnapshot);
  Serial.print(",");
  Serial.println(invalidSnapshot);
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(clkPin, INPUT_PULLUP);
  pinMode(dtPin, INPUT_PULLUP);

  lastState = (digitalRead(clkPin) << 1) | digitalRead(dtPin);

  enableInterrupt(clkPin, handleEncoder, CHANGE);
  enableInterrupt(dtPin, handleEncoder, CHANGE);

  Wire.begin();
  Wire.setClock(400000);

  if (!mcp.begin()) {
    Serial.println("ERR,MCP4728_NOT_FOUND");
    while (1) {}
  }

  measuredAngleDeg = getSteeringAngleDeg(0);
  prevAngleDeg = measuredAngleDeg;

  applyDelta(0.0);

  Serial.println("READY");
  Serial.println("DATA FORMAT:");
  Serial.println("DATA,t_ms,target_deg,angle_deg,error_deg,delta_v,va_out,vb_out,codeA,codeB,count,pid_on,p_term,i_term,d_term,isr,invalid");
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  handleSerial();

  unsigned long now = millis();

  if (now - lastControlMs >= controlPeriodMs) {
    lastControlMs = now;
    runControl();
  }

  if (now - lastPrintMs >= printIntervalMs) {
    lastPrintMs = now;
    printDataLine();
  }
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
    lastPTerm = 0.0;
    lastITerm = 0.0;
    lastDTerm = 0.0;
    return;
  }

  float dt = controlPeriodMs / 1000.0f;

  float error = targetAngleDeg - measuredAngleDeg;
  float dMeas = (measuredAngleDeg - prevAngleDeg) / dt;

  integralTerm += error * dt;

  if (integralTerm > integralLimit) integralTerm = integralLimit;
  if (integralTerm < -integralLimit) integralTerm = -integralLimit;

  lastPTerm = Kp * error;
  lastITerm = Ki * integralTerm;
  lastDTerm = -Kd * dMeas;

  float u = lastPTerm + lastITerm + lastDTerm;

  if (u > DELTA_LIMIT) u = DELTA_LIMIT;
  if (u < -DELTA_LIMIT) u = -DELTA_LIMIT;

  applyDelta(u);

  lastErrorDeg = error;
  prevAngleDeg = measuredAngleDeg;
}

// =========================
// ENCODER ISR
// =========================
void handleEncoder() {
  uint8_t clkState = (PINC & (1 << PC3)) ? 1 : 0;
  uint8_t dtState  = (PINC & (1 << PC1)) ? 1 : 0;

  uint8_t newState = (clkState << 1) | dtState;
  uint8_t transition = (lastState << 2) | newState;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      counter++;
      isrEvents++;
      break;

    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      counter--;
      isrEvents++;
      break;

    default:
      if (newState != lastState) {
        invalidTransitions++;
      }
      break;
  }

  lastState = newState;
}

// =========================
// SERIAL COMMANDS
// =========================
void handleSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();

  if (s.length() == 0) return;

  if (s == "e" || s == "E") {
    pidEnabled = true;
    resetPID();
    Serial.println("INFO,PID_ENABLED");
    return;
  }

  if (s == "d" || s == "D") {
    pidEnabled = false;
    stopOutput();
    Serial.println("INFO,PID_DISABLED");
    return;
  }

  if (s == "z" || s == "Z") {
    noInterrupts();
    counter = 0;
    invalidTransitions = 0;
    isrEvents = 0;
    interrupts();

    measuredAngleDeg = 0.0;
    prevAngleDeg = 0.0;
    resetPID();

    Serial.println("INFO,ENCODER_ZEROED");
    return;
  }

  if (s == "p" || s == "P") {
    printDataLine();
    return;
  }

  if (s == "r" || s == "R") {
    encoderCwRight = !encoderCwRight;
    Serial.print("INFO,ENCODER_REVERSED,");
    Serial.println(encoderCwRight ? 1 : 0);
    return;
  }

  if (s.startsWith("s")) {
    float newTarget = s.substring(1).toFloat();
    if (abs(newTarget - targetAngleDeg) > 5.0) {
      integralTerm = 0.0;
    }
    targetAngleDeg = newTarget;
    Serial.print("INFO,TARGET_DEG,");
    Serial.println(targetAngleDeg, 3);
    return;
  }

  if (s.startsWith("kp")) {
    Kp = s.substring(2).toFloat();
    Serial.print("INFO,KP,");
    Serial.println(Kp, 6);
    return;
  }

  if (s.startsWith("ki")) {
    Ki = s.substring(2).toFloat();
    Serial.print("INFO,KI,");
    Serial.println(Ki, 6);
    return;
  }

  if (s.startsWith("kd")) {
    Kd = s.substring(2).toFloat();
    Serial.print("INFO,KD,");
    Serial.println(Kd, 6);
    return;
  }
}
