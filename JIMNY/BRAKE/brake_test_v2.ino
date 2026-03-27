/*
 * brake_test_v2.ino
 *
 * Scalable brake test for NEMA-34 via D0860HA.
 * All key parameters (speed, max turns, home origin) are
 * configurable at runtime via Serial commands — no reupload needed.
 *
 * Pins (same as steerbok.ino):
 *   PUL = 5, DIR = 4, ENA = 3
 *
 * DIP switch microstep setting must match MICROSTEPS below.
 * Default: 1600 steps/rev (SW5=ON SW6=OFF SW7=ON SW8=ON)
 *
 * Serial commands (115200 baud):
 *   B:0-100      — brake to % of max travel
 *   H            — go to home position (step 0)
 *   SETHOME      — declare current position as home (step 0)
 *   OFF          — disable motor coils
 *   JOG:+N       — jog N steps forward (e.g. JOG:+50)
 *   JOG:-N       — jog N steps backward (e.g. JOG:-50)
 *   SPEED:N      — set max speed in steps/sec (e.g. SPEED:400)
 *   ACCEL:N      — set acceleration in steps/sec² (e.g. ACCEL:300)
 *   TURNS:N.NN   — set max travel in motor turns (e.g. TURNS:0.85)
 *   CONFIG       — print current configuration
 *   POS          — print current position
 */

#include <AccelStepper.h>

// ── Fixed hardware config ──────────────────────────────────────
const int PUL_PIN    = 5;
const int DIR_PIN    = 4;
const int ENA_PIN    = 3;
const int MICROSTEPS = 1600;   // must match DIP switch SW5-8

// ── Runtime config (all changeable via Serial) ─────────────────
float  cfgMaxTurns  = 0.80;    // motor revolutions = full brake travel
float  cfgMaxSpeed  = 600.0;   // steps/sec
float  cfgAccel     = 400.0;   // steps/sec²

// Derived — recalculated whenever cfgMaxTurns changes
long   maxSteps     = 0;

// ── Stepper ────────────────────────────────────────────────────
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

// ── State ──────────────────────────────────────────────────────
bool  motorEnabled  = false;
long  targetSteps   = 0;

// ── Helpers ────────────────────────────────────────────────────
void recalcMaxSteps() {
  maxSteps = (long)(MICROSTEPS * cfgMaxTurns);
}

void applyConfig() {
  stepper.setMaxSpeed(cfgMaxSpeed);
  stepper.setAcceleration(cfgAccel);
  recalcMaxSteps();
}

void enableMotor() {
  if (!motorEnabled) {
    stepper.enableOutputs();
    motorEnabled = true;
    Serial.println("Motor ENABLED");
  }
}

void printConfig() {
  Serial.println("--- CONFIG ---");
  Serial.print("MICROSTEPS : "); Serial.println(MICROSTEPS);
  Serial.print("MAX_TURNS  : "); Serial.println(cfgMaxTurns, 3);
  Serial.print("MAX_STEPS  : "); Serial.println(maxSteps);
  Serial.print("MAX_SPEED  : "); Serial.print(cfgMaxSpeed, 0); Serial.println(" steps/sec");
  Serial.print("ACCEL      : "); Serial.print(cfgAccel, 0);    Serial.println(" steps/sec²");
  Serial.print("CURR_POS   : "); Serial.println(stepper.currentPosition());
  Serial.println("--------------");
}

void printPos() {
  long pos = stepper.currentPosition();
  float pct = maxSteps > 0 ? (float)pos / maxSteps * 100.0f : 0;
  Serial.print("POS:");    Serial.print(pos);
  Serial.print(" PCT:");   Serial.print(pct, 1);
  Serial.print(" TARGET:"); Serial.println(targetSteps);
}

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  stepper.setEnablePin(ENA_PIN);
  stepper.setPinsInverted(false, false, true); // ENA active LOW
  stepper.disableOutputs();
  stepper.setCurrentPosition(0);

  recalcMaxSteps();
  applyConfig();

  Serial.println("=== BRAKE TEST v2 READY ===");
  printConfig();
  Serial.println("Commands: B:% | H | SETHOME | OFF | JOG:+N | JOG:-N | SPEED:N | ACCEL:N | TURNS:N | CONFIG | POS");
}

// ── Loop ───────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) handleCommand(cmd);
  }

  if (motorEnabled) stepper.run();

  // Periodic position report
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 200) {
    lastReport = millis();
    printPos();
  }
}

// ── Command handler ────────────────────────────────────────────
void handleCommand(String raw) {
  String cmd = raw;
  cmd.toUpperCase();

  // B:0-100 — brake percentage
  if (cmd.startsWith("B:")) {
    int pct = constrain(cmd.substring(2).toInt(), 0, 100);
    targetSteps = (long)(maxSteps * pct / 100.0);
    enableMotor();
    stepper.moveTo(targetSteps);
    Serial.print("BRAKE:"); Serial.print(pct);
    Serial.print("% -> "); Serial.print(targetSteps); Serial.println(" steps");
    return;
  }

  // JOG:+N or JOG:-N — relative move for finding home
  if (cmd.startsWith("JOG:")) {
    String valStr = cmd.substring(4);
    int jogSteps = valStr.toInt();  // handles +/- prefix
    enableMotor();
    long newTarget = stepper.currentPosition() + jogSteps;
    stepper.moveTo(newTarget);
    targetSteps = newTarget;
    Serial.print("JOG:"); Serial.print(jogSteps);
    Serial.print(" -> pos "); Serial.println(newTarget);
    return;
  }

  // SPEED:N
  if (cmd.startsWith("SPEED:")) {
    float v = cmd.substring(6).toFloat();
    if (v > 0 && v <= 3000) {
      cfgMaxSpeed = v;
      stepper.setMaxSpeed(cfgMaxSpeed);
      Serial.print("SPEED set to "); Serial.println(cfgMaxSpeed, 0);
    } else {
      Serial.println("ERR: SPEED must be 1–3000");
    }
    return;
  }

  // ACCEL:N
  if (cmd.startsWith("ACCEL:")) {
    float v = cmd.substring(6).toFloat();
    if (v > 0 && v <= 5000) {
      cfgAccel = v;
      stepper.setAcceleration(cfgAccel);
      Serial.print("ACCEL set to "); Serial.println(cfgAccel, 0);
    } else {
      Serial.println("ERR: ACCEL must be 1–5000");
    }
    return;
  }

  // TURNS:N.NN
  if (cmd.startsWith("TURNS:")) {
    float v = cmd.substring(6).toFloat();
    if (v > 0.0 && v <= 5.0) {
      cfgMaxTurns = v;
      recalcMaxSteps();
      Serial.print("TURNS set to "); Serial.print(cfgMaxTurns, 3);
      Serial.print(" -> MAX_STEPS="); Serial.println(maxSteps);
    } else {
      Serial.println("ERR: TURNS must be 0.01–5.0");
    }
    return;
  }

  // SETHOME — declare current position as origin
  if (cmd == "SETHOME") {
    stepper.setCurrentPosition(0);
    targetSteps = 0;
    Serial.print("HOME SET at previous step ");
    Serial.println(stepper.currentPosition()); // will be 0
    Serial.println("Origin is now here.");
    return;
  }

  // H — go to home
  if (cmd == "H") {
    enableMotor();
    targetSteps = 0;
    stepper.moveTo(0);
    Serial.println("Homing to 0...");
    return;
  }

  // OFF — disable coils
  if (cmd == "OFF") {
    stepper.disableOutputs();
    motorEnabled = false;
    Serial.println("Motor DISABLED — coils off, shaft free");
    return;
  }

  // CONFIG
  if (cmd == "CONFIG") {
    printConfig();
    return;
  }

  // POS
  if (cmd == "POS") {
    printPos();
    return;
  }

  Serial.print("ERR: unknown command: "); Serial.println(raw);
}
