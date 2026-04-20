#include <SPI.h>
#include <mcp2515.h>
#include <AccelStepper.h>
#include "can_ids.h"

const int PUL_PIN    = 5;
const int DIR_PIN    = 4;
const int ENA_PIN    = 3;
const int MICROSTEPS = 1600;

const int LED_CAN   = 8;
const int LED_BRAKE = 6;
const int LED_READY = 7;

MCP2515 mcp2515(10);
struct can_frame canMsg;
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

float  cfgMaxTurns = 0.80;
float  cfgMaxSpeed = 600.0;
float  cfgAccel    = 400.0;
long   maxSteps    = 0;

bool  motorEnabled = false;
bool  disableWhenReleased = false;

long  targetSteps  = 0;
float lastBrakePct = 0.0;
int   mode         = 0;

uint32_t heartbeatInterval   = 200;
unsigned long lastHbReceived = 0;
unsigned long lastHbSent     = 0;
unsigned long lastCanSig     = 0;
const unsigned long CAN_LED_DUR = 50;

// #define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
  #define Debug Serial
#else
  #define Debug if(false) Serial
#endif

void recalcMaxSteps() {
  maxSteps = (long)(MICROSTEPS * cfgMaxTurns);
}

void enableMotor() {
  if (!motorEnabled) {
    stepper.enableOutputs();
    motorEnabled = true;
  }
}

void disableMotor() {
  if (motorEnabled) {
    stepper.disableOutputs();
    motorEnabled = false;
  }
}

void commandBrakePct(float pct, bool disableAtZero = false) {
  pct = constrain(pct, 0.0, 100.0);

  lastBrakePct = pct;
  targetSteps  = (long)(maxSteps * pct / 100.0);

  disableWhenReleased = (disableAtZero && targetSteps == 0);

  enableMotor();
  stepper.moveTo(targetSteps);

  Debug.print("BRAKE CMD: ");
  Debug.println(pct, 1);
}

void enterEmergencyBrake(const char* reason) {
  disableWhenReleased = false;
  commandBrakePct(100.0, false);

  Serial.print(reason);
  Serial.println(": FULL BRAKE 100%");
}

void releaseBrakeForManual(const char* reason) {
  commandBrakePct(0.0, true);

  Serial.print(reason);
  Serial.println(": RELEASE BRAKE -> 0%");
}

void sendHeartbeat() {
  struct can_frame hb;
  hb.can_id  = CAN_HB_BRAKE;   // 0x13
  hb.can_dlc = 8;

  float posF = (float)stepper.currentPosition();
  float pctF = lastBrakePct;

  memcpy(hb.data,     &posF, 4);
  memcpy(hb.data + 4, &pctF, 4);

  mcp2515.sendMessage(&hb);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_CAN,   OUTPUT);
  pinMode(LED_BRAKE, OUTPUT);
  pinMode(LED_READY, OUTPUT);
  digitalWrite(LED_CAN,   LOW);
  digitalWrite(LED_BRAKE, LOW);
  digitalWrite(LED_READY, LOW);

  stepper.setEnablePin(ENA_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.disableOutputs();
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(cfgMaxSpeed);
  stepper.setAcceleration(cfgAccel);
  recalcMaxSteps();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  lastHbReceived = millis();
  lastHbSent     = millis();

  Serial.println("=== BRAKE CAN READY ===");
  Serial.println("Serial: B:% H SETHOME OFF JOG:+N SPEED:N ACCEL:N TURNS:N CONFIG POS");
}

void loop() {
  unsigned long now = millis();

  // ── CAN receive ───────────────────────────────────────────
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    digitalWrite(LED_CAN, HIGH);
    lastCanSig = now;

    if (canMsg.can_id == CAN_HB_STERFBOARD) {   // 0x01
      heartbeatInterval = (canMsg.data[0] << 8) | canMsg.data[1];
      int newMode       = canMsg.data[2];
      int oldMode       = mode;

      mode = newMode;
      lastHbReceived = now;

      if (mode == 2) {
        if (oldMode != 2 || lastBrakePct < 99.9) {
          enterEmergencyBrake("MODE 2");
        }
      }
      else if (mode == 0) {
        if (oldMode != 0) {
          releaseBrakeForManual("ENTER MANUAL");
        }
      }
    }
    else if (canMsg.can_id == CAN_BRAKE_PCT) {   // 0x130
      float rxPct;
      memcpy(&rxPct, canMsg.data, 4);

      if (mode == 1) {
        commandBrakePct(rxPct, rxPct <= 0.01);
      } else {
        Debug.println("Ignored brake command — not AUTO");
      }
    }
  }

  // ── Watchdog ──────────────────────────────────────────────
  if (now - lastHbReceived > heartbeatInterval * 2) {
    if (mode != 2 || lastBrakePct < 99.9) {
      mode = 2;
      enterEmergencyBrake("WATCHDOG");
    }
  }

  // ── Motion ────────────────────────────────────────────────
  if (motorEnabled) {
    stepper.run();

    if (disableWhenReleased && stepper.distanceToGo() == 0) {
      disableMotor();
      disableWhenReleased = false;
      Serial.println("BRAKE RELEASE COMPLETE: motor disabled");
    }
  }

  // ── Heartbeat out ─────────────────────────────────────────
  if (now - lastHbSent >= heartbeatInterval) {
    sendHeartbeat();
    lastHbSent = now;
  }

  // ── Serial debug commands ─────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) handleSerialCommand(cmd);
  }

  // ── LEDs ──────────────────────────────────────────────────
  if (now - lastCanSig > CAN_LED_DUR) digitalWrite(LED_CAN, LOW);
  digitalWrite(LED_BRAKE, lastBrakePct > 5.0 ? HIGH : LOW);
  digitalWrite(LED_READY, mode == 1 ? HIGH : LOW);

  // ── Position report ───────────────────────────────────────
  static unsigned long lastReport = 0;
  if (now - lastReport > 500) {
    lastReport = now;
    long pos  = stepper.currentPosition();
    float pct = maxSteps > 0 ? (float)pos / maxSteps * 100.0f : 0.0f;

    Serial.print("POS:");  Serial.print(pos);
    Serial.print(" PCT:"); Serial.print(pct, 1);
    Serial.print(" BRK:"); Serial.print(lastBrakePct, 1);
    Serial.print(" MODE:"); Serial.println(mode);
  }
}

void handleSerialCommand(String raw) {
  String cmd = raw;
  cmd.toUpperCase();

  if (cmd.startsWith("B:")) {
    float pct = constrain(cmd.substring(2).toFloat(), 0, 100);
    commandBrakePct(pct, pct <= 0.01);
    return;
  }

  if (cmd.startsWith("JOG:")) {
    int j = cmd.substring(4).toInt();
    enableMotor();
    disableWhenReleased = false;
    long t = stepper.currentPosition() + j;
    stepper.moveTo(t);
    targetSteps = t;
    return;
  }

  if (cmd.startsWith("SPEED:")) {
    float v = cmd.substring(6).toFloat();
    if (v > 0 && v <= 3000) {
      cfgMaxSpeed = v;
      stepper.setMaxSpeed(v);
    }
    return;
  }

  if (cmd.startsWith("ACCEL:")) {
    float v = cmd.substring(6).toFloat();
    if (v > 0 && v <= 5000) {
      cfgAccel = v;
      stepper.setAcceleration(v);
    }
    return;
  }

  if (cmd.startsWith("TURNS:")) {
    float v = cmd.substring(6).toFloat();
    if (v > 0 && v <= 5.0) {
      cfgMaxTurns = v;
      recalcMaxSteps();
    }
    return;
  }

  if (cmd == "SETHOME") {
    stepper.setCurrentPosition(0);
    targetSteps = 0;
    Serial.println("HOME SET");
    return;
  }

  if (cmd == "H") {
    commandBrakePct(0.0, true);
    return;
  }

  if (cmd == "OFF") {
    disableWhenReleased = false;
    disableMotor();
    return;
  }

  if (cmd == "CONFIG") {
    Serial.print("TURNS:"); Serial.print(cfgMaxTurns, 3);
    Serial.print(" STEPS:"); Serial.print(maxSteps);
    Serial.print(" SPEED:"); Serial.print(cfgMaxSpeed, 0);
    Serial.print(" ACCEL:"); Serial.println(cfgAccel, 0);
    return;
  }

  if (cmd == "POS") return;

  Serial.print("ERR: ");
  Serial.println(raw);
}
