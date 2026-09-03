#include <SPI.h>
#include <mcp2515.h>
#include "can_ids.h"

MCP2515 mcp2515(10);

const int relay48v    = 3;
const int relay9v     = 4;
const int eButton     = 5;
const int autoButton  = 6;
const int res1Button  = 7;
const int res2Button  = 8;
const int canLED      = 9;
unsigned long canLEDtime = 0;
const int cubePWM     = A0;

const uint32_t heartbeatInterval = 200;  // 5 Hz
const uint32_t commandInterval   = 20;   // 50 Hz

const bool ENABLE_BOARD_FAULTS   = true;
const uint8_t BOARD_TIMEOUT_MULT = 4;

// ===== DEBUG CSV =====
const bool DEBUG_CSV_ENABLED = true;

unsigned long serialCmdCount   = 0;
unsigned long serialHbCount    = 0;
unsigned long canSteerTxCount  = 0;

float lastTargetJumpDeg        = 0.0;
float lastSteeringAngleSent    = 0.0;
float lastSteeringRateSent     = 0.0;
// =====================

unsigned long nowMS                   = millis();
unsigned long lastHeartbeatTime       = 0;
unsigned long lastCommandTime         = 0;
unsigned long lastSerialHeartbeatTime = nowMS;
bool serialHeartbeatReceived          = false;

bool ackSignalInvalid = true;
unsigned long lastAckTime  = nowMS;
unsigned long noAckTimeout = 500;
bool noAckReceived = true;
int hbSignal = 0;

unsigned long lastCanSig      = nowMS;
const unsigned long canLEDDur = 50;

float steeringAngle, steeringAngleVelocity, velocity,
      acceleration, jerk, gpsVelocity;

float velocitySetpoint       = 0.0;
float velocityFeedback       = 0.0;
float steeringRateSetPoint   = 0.0;
float steeringAngleFeedback  = 0.0;

float brakePositionFeedback  = 0.0;
float brakePctFeedback       = 0.0;

unsigned long lastAxelBrakeFeedback = 0;
unsigned long lastSteeringFeedback  = 0;
unsigned long lastBrakeFeedback     = 0;

bool axelBrakeAlive = false;
bool steeringAlive  = false;
bool brakeAlive     = false;

enum HbSignal  { DISARMED = 0, VALID_ARMED = 1, INVALID_ARMED = 2 };
enum Mode      { MANUAL = 0, AUTO = 1, EMERGENCY = 2 };

#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
  #define Debug Serial
#else
  #define Debug if (false) Serial
#endif

void sendHeartbeatMessage(int mode);
void sendCANMessages();
void checkCANMessages();
void led_maintenance();
void parseSerialData(String inputData);
void floatToBytes(float value, uint8_t* bytes);
void updateBoardAliveFlags(unsigned long now);
bool anyCriticalBoardMissing();
void sendSerialFeedbackToJetson();
void handleSerialInput(unsigned long now);
int computeCheckedMode(unsigned long now);
void sendDebugCsv(unsigned long now, int checkedMode);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  pinMode(relay48v,    OUTPUT);
  pinMode(relay9v,     OUTPUT);
  pinMode(eButton,     INPUT);
  pinMode(autoButton,  INPUT_PULLUP);
  pinMode(res1Button,  INPUT_PULLUP);
  pinMode(res2Button,  INPUT_PULLUP);
  pinMode(canLED,      OUTPUT);

  digitalWrite(relay9v,  LOW);
  digitalWrite(relay48v, HIGH);

  if (DEBUG_CSV_ENABLED) {
    Serial.println("DBG_HEADER,t_ms,hbSignal,checkedMode,serialHbOk,noAck,ackAgeMs,steerCmdRx,steerRateRx,targetJump,steerCmdCan,steerRateCan,steerFb,steerRateFb,axelAlive,steeringAlive,brakeAlive,serialCmdCount,serialHbCount,canSteerTxCount");
  }
}

void loop() {
  unsigned long now = millis();

  checkCANMessages();
  updateBoardAliveFlags(now);

  if (digitalRead(eButton) == LOW) {
    digitalWrite(relay48v, LOW);

    if (now - lastHeartbeatTime >= heartbeatInterval) {
      sendHeartbeatMessage(EMERGENCY);
      lastHeartbeatTime = now;
    }

    led_maintenance();
    return;
  }

  handleSerialInput(now);

  if (now - lastSerialHeartbeatTime >= 2 * heartbeatInterval) {
    serialHeartbeatReceived = false;
  }

  now = millis();
  updateBoardAliveFlags(now);

  int checkedMode = computeCheckedMode(now);

  digitalWrite(relay48v, checkedMode == AUTO ? HIGH : LOW);

  if (now - lastHeartbeatTime >= heartbeatInterval) {
    sendHeartbeatMessage(checkedMode);
    lastHeartbeatTime = now;
  }

  if (now - lastCommandTime >= commandInterval) {
    sendCANMessages();
    lastCommandTime = now;
  }

  led_maintenance();
}

int computeCheckedMode(unsigned long now) {
  if ((hbSignal == VALID_ARMED) && (now - lastAckTime > noAckTimeout)) {
    noAckReceived = true;
  } else {
    noAckReceived = false;
  }

  int checkedMode = MANUAL;

  switch (hbSignal) {
    case DISARMED:
      checkedMode = MANUAL;
      break;

    case VALID_ARMED:
      checkedMode = AUTO;
      break;

    case INVALID_ARMED:
      checkedMode = EMERGENCY;
      break;

    default:
      checkedMode = MANUAL;
      break;
  }

  if (checkedMode == AUTO && noAckReceived) {
    checkedMode = EMERGENCY;
    Debug.println("FAULT: command timeout -> EMERGENCY");
  }

  static bool boardFaultLatched = false;

  if (ENABLE_BOARD_FAULTS) {
    if (boardFaultLatched && (hbSignal == DISARMED || !anyCriticalBoardMissing())) {
      boardFaultLatched = false;
      Debug.println("INFO: board latch cleared");
    }

    if (checkedMode == AUTO && anyCriticalBoardMissing()) {
      boardFaultLatched = true;
      Debug.print("FAULT: missing board feedback -> EMERGENCY | axel=");
      Debug.print(axelBrakeAlive);
      Debug.print(" steering=");
      Debug.print(steeringAlive);
      Debug.print(" brake=");
      Debug.println(brakeAlive);
    }

    if (boardFaultLatched && checkedMode == AUTO) {
      checkedMode = EMERGENCY;
    }
  }

  return checkedMode;
}

void handleSerialInput(unsigned long now) {
  if (Serial.available() <= 0) return;

  String inputData = Serial.readStringUntil('\n');
  inputData.trim();

  if (inputData.length() == 0) return;
  if (inputData[0] != '<') return;
  if (inputData[inputData.length() - 1] != '>') return;

  inputData.remove(0, 1);
  inputData.remove(inputData.length() - 1, 1);

  if (inputData.startsWith("H,")) {
    hbSignal = inputData.substring(2).toInt();
    serialHbCount++;
    lastSerialHeartbeatTime = now;
    serialHeartbeatReceived = true;

    // Keep old working behaviour:
    // heartbeat also keeps the serial command path alive.
    lastAckTime = now;
  } else {
    parseSerialData(inputData);
  }
}

void parseSerialData(String inputData) {
  float values[6];
  int index = 0;

  char buffer[96];
  inputData.toCharArray(buffer, sizeof(buffer));
  char* token = strtok(buffer, ",");

  while (token != NULL && index < 6) {
    values[index++] = atof(token);
    token = strtok(NULL, ",");
  }

  if (index == 6) {
    float newSteer = values[0];

    lastTargetJumpDeg = newSteer - steeringAngle;
    serialCmdCount++;

    lastAckTime           = millis();
    steeringAngle         = newSteer;
    steeringAngleVelocity = values[1];
    velocity              = values[2];
    acceleration          = values[3];
    jerk                  = values[4];
    gpsVelocity           = values[5];
  }
}

void sendHeartbeatMessage(int mode) {
  struct can_frame hb;

  hb.can_id  = CAN_HB_STERFBOARD;
  hb.can_dlc = 4;

  hb.data[0] = heartbeatInterval >> 8;
  hb.data[1] = heartbeatInterval & 0xFF;
  hb.data[2] = (mode == EMERGENCY) ? 0x02 : (mode == AUTO) ? 0x01 : 0x00;
  hb.data[3] = 0x00;

  mcp2515.sendMessage(&hb);

  sendSerialFeedbackToJetson();
  sendDebugCsv(millis(), mode);
}

void sendSerialFeedbackToJetson() {
  Serial.print("<");
  Serial.print(velocitySetpoint);     Serial.print(", ");
  Serial.print(velocityFeedback);     Serial.print(", ");

  // Steering feedback is no longer controlled by sterfBoard.
  // Keep the old 4-field format for compatibility with old mode_switch/to_vehicle.
  Serial.print(0.0);                  Serial.print(", ");
  Serial.print(0.0);
  Serial.println(">");
}

void sendDebugCsv(unsigned long now, int checkedMode) {
  if (!DEBUG_CSV_ENABLED) return;

  Serial.print("DBG,");
  Serial.print(now);                               Serial.print(",");
  Serial.print(hbSignal);                          Serial.print(",");
  Serial.print(checkedMode);                       Serial.print(",");
  Serial.print(serialHeartbeatReceived ? 1 : 0);   Serial.print(",");
  Serial.print(noAckReceived ? 1 : 0);             Serial.print(",");
  Serial.print(now - lastAckTime);                 Serial.print(",");
  Serial.print(steeringAngle, 3);                  Serial.print(",");
  Serial.print(steeringAngleVelocity, 3);          Serial.print(",");
  Serial.print(lastTargetJumpDeg, 3);              Serial.print(",");
  Serial.print(lastSteeringAngleSent, 3);          Serial.print(",");
  Serial.print(lastSteeringRateSent, 3);           Serial.print(",");
  Serial.print(steeringAngleFeedback, 3);          Serial.print(",");
  Serial.print(steeringRateSetPoint, 3);           Serial.print(",");
  Serial.print(axelBrakeAlive ? 1 : 0);            Serial.print(",");
  Serial.print(steeringAlive ? 1 : 0);             Serial.print(",");
  Serial.print(brakeAlive ? 1 : 0);                Serial.print(",");
  Serial.print(serialCmdCount);                    Serial.print(",");
  Serial.print(serialHbCount);                     Serial.print(",");
  Serial.println(canSteerTxCount);
}

void sendCANMessages() {
  // New 2-USB architecture:
  // sterfBoard must NOT command steering anymore.
  // Steering is handled by laptop/Jetson -> USB1 -> steering board.
  lastSteeringAngleSent = 0.0;
  lastSteeringRateSent  = 0.0;

  struct can_frame velocityMsg;
  velocityMsg.can_id  = CAN_VELOCITY_TARGET;
  velocityMsg.can_dlc = 8;
  floatToBytes(velocity,     velocityMsg.data);
  floatToBytes(acceleration, &velocityMsg.data[4]);
  mcp2515.sendMessage(&velocityMsg);

  struct can_frame gpsMsg;
  gpsMsg.can_id  = CAN_GPS_VELOCITY;
  gpsMsg.can_dlc = 4;
  floatToBytes(gpsVelocity, gpsMsg.data);
  mcp2515.sendMessage(&gpsMsg);
}

void floatToBytes(float value, uint8_t* bytes) {
  union {
    float f;
    uint8_t b[4];
  } c;

  c.f = value;
  memcpy(bytes, c.b, 4);
}

void checkCANMessages() {
  struct can_frame msg;

  while (mcp2515.readMessage(&msg) == MCP2515::ERROR_OK) {
    if (msg.can_dlc != 8) continue;

    digitalWrite(canLED, HIGH);
    lastCanSig = millis();

    if (msg.can_id == CAN_HB_AXELBRAKE) {
      memcpy(&velocitySetpoint, &msg.data[0], 4);
      memcpy(&velocityFeedback, &msg.data[4], 4);
      lastAxelBrakeFeedback = lastCanSig;
    }
    else if (msg.can_id == CAN_HB_STEERING) {
      // Not required anymore, but keep it for debug if the steering board is still connected.
      memcpy(&steeringRateSetPoint,  &msg.data[0], 4);
      memcpy(&steeringAngleFeedback, &msg.data[4], 4);
      lastSteeringFeedback = lastCanSig;
    }
    else if (msg.can_id == CAN_HB_BRAKE) {
      memcpy(&brakePositionFeedback, &msg.data[0], 4);
      memcpy(&brakePctFeedback,      &msg.data[4], 4);
      lastBrakeFeedback = lastCanSig;
    }
  }
}

void updateBoardAliveFlags(unsigned long now) {
  unsigned long timeoutMs = (unsigned long)heartbeatInterval * BOARD_TIMEOUT_MULT;

  axelBrakeAlive = (now - lastAxelBrakeFeedback) <= timeoutMs;
  steeringAlive  = (now - lastSteeringFeedback)  <= timeoutMs;
  brakeAlive     = (now - lastBrakeFeedback)     <= timeoutMs;
}

bool anyCriticalBoardMissing() {
  // New 2-USB architecture:
  // steering board is NOT part of sterfBoard CAN safety anymore.
  // Only axelBrake and brake board remain critical on this CAN bus.
  return (!axelBrakeAlive || !brakeAlive);
}

void led_maintenance() {
  nowMS = millis();

  if (nowMS - lastCanSig > canLEDDur) {
    digitalWrite(canLED, LOW);
  }
}
