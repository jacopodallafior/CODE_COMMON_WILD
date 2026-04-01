#include <SPI.h>
#include <mcp2515.h>
#include "can_ids.h"

MCP2515 mcp2515(10);

const int relay48v   = 3;
const int relay9v    = 4;
const int eButton    = 5;
const int autoButton = 6;
const int res1Button = 7;
const int res2Button = 8;
const int canLED     = 9;
const int cubePWM    = A0;

const uint32_t heartbeatInterval = 200;
unsigned long nowMS = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastSerialHeartbeatTime = 0;
bool serialHeartbeatReceived = false;

bool ackSignalInvalid = true;
unsigned long lastAckTime = 0;
unsigned long noAckTimeout = 500;
bool noAckReceived = true;
int hbSignal = 0;

unsigned long lastCanSig = 0;
const unsigned long canLEDDur = 50;

float steeringAngle = 0.0f;
float steeringAngleVelocity = 0.0f;
float velocity = 0.0f;
float acceleration = 0.0f;
float jerk = 0.0f;
float gpsVelocity = 0.0f;

float velocitySetpoint = 0.0f;
float velocityFeedback = 0.0f;
float steeringRateSetPoint = 0.0f;
float steeringAngleFeedback = 0.0f;

enum HbSignal { DISARMED = 0, VALID_ARMED = 1, INVALID_ARMED = 2 };
enum Mode { MANUAL = 0, AUTO = 1, EMERGENCY = 2 };

#define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
  #define Debug Serial
#else
  #define Debug if (false) Serial
#endif

void setup() {
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  pinMode(relay48v, OUTPUT);
  pinMode(relay9v, OUTPUT);
  pinMode(eButton, INPUT);
  pinMode(autoButton, INPUT_PULLUP);
  pinMode(res1Button, INPUT_PULLUP);
  pinMode(res2Button, INPUT_PULLUP);
  pinMode(canLED, OUTPUT);

  digitalWrite(relay9v, LOW);
  digitalWrite(relay48v, HIGH);
  digitalWrite(canLED, LOW);

  nowMS = millis();
  lastHeartbeatTime = nowMS;
  lastSerialHeartbeatTime = nowMS;
  lastAckTime = nowMS;
  lastCanSig = nowMS;

  Debug.println("STERFBOARD READY");
  Debug.println("AUTO BUTTON TEMPORARILY IGNORED");
}

void loop() {
  unsigned long now = millis();

  if (digitalRead(eButton) == LOW) {
    if (digitalRead(relay48v) == HIGH) {
      Debug.println("BUTTON: EMERGENCY");
      sendHeartbeatMessage(EMERGENCY);
      lastHeartbeatTime = now;
      digitalWrite(relay48v, LOW);
    } else if (now - lastHeartbeatTime >= heartbeatInterval) {
      Debug.println("BUTTON: EMERGENCY");
      sendHeartbeatMessage(EMERGENCY);
      lastHeartbeatTime = now;
    }
  }
  else {
    handleSerialInput(now);

    if (now - lastSerialHeartbeatTime >= 2 * heartbeatInterval) {
      serialHeartbeatReceived = false;
    }

    if (now - lastHeartbeatTime >= heartbeatInterval) {
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

      Debug.print("STATE hbSignal=");
      Debug.print(hbSignal);
      Debug.print(" checkedMode=");
      Debug.print(checkedMode);
      Debug.print(" serialHB=");
      Debug.print(serialHeartbeatReceived ? 1 : 0);
      Debug.print(" noAck=");
      Debug.print(noAckReceived ? 1 : 0);
      Debug.print(" relay48v=");
      Debug.println(checkedMode == AUTO ? 1 : 0);

      digitalWrite(relay48v, checkedMode == AUTO ? HIGH : LOW);

      sendHeartbeatMessage(checkedMode);
      lastHeartbeatTime = now;

      sendCANMessages();
    }

    checkCANMessages();
    led_maintenance();
  }
}

void handleSerialInput(unsigned long now) {
  if (Serial.available() <= 0) return;

  String inputData = Serial.readStringUntil('\n');
  inputData.trim();

  Debug.print("GOT:[");
  Debug.print(inputData);
  Debug.println("]");

  if (inputData.length() == 0) return;
  if (inputData[0] != '<') {
    Debug.println("ERR: missing <");
    return;
  }
  if (inputData[inputData.length() - 1] != '>') {
    Debug.println("ERR: missing >");
    return;
  }

  inputData.remove(0, 1);
  inputData.remove(inputData.length() - 1, 1);

  Debug.print("PARSED:[");
  Debug.print(inputData);
  Debug.println("]");

  if (inputData.startsWith("H,")) {
    hbSignal = inputData.substring(2).toInt();
    lastSerialHeartbeatTime = now;
    serialHeartbeatReceived = true;

    Debug.print("HB SIGNAL SET TO ");
    Debug.println(hbSignal);
  } else {
    parseSerialData(inputData);
  }
}

void parseSerialData(String inputData) {
  float values[6] = {0, 0, 0, 0, 0, 0};
  int index = 0;

  char buffer[120];
  inputData.toCharArray(buffer, sizeof(buffer));

  char* token = strtok(buffer, ",");
  while (token != NULL && index < 6) {
    values[index++] = atof(token);
    token = strtok(NULL, ",");
  }

  if (index == 6) {
    lastAckTime = millis();
    steeringAngle = values[0];
    steeringAngleVelocity = values[1];
    velocity = values[2];
    acceleration = values[3];
    jerk = values[4];
    gpsVelocity = values[5];

    Debug.print("ACK DATA SA=");
    Debug.print(steeringAngle);
    Debug.print(" SAV=");
    Debug.print(steeringAngleVelocity);
    Debug.print(" V=");
    Debug.print(velocity);
    Debug.print(" A=");
    Debug.print(acceleration);
    Debug.print(" J=");
    Debug.print(jerk);
    Debug.print(" GPSV=");
    Debug.println(gpsVelocity);
  } else {
    Debug.print("ERR: expected 6 floats, got ");
    Debug.println(index);
  }
}

void sendHeartbeatMessage(int mode) {
  struct can_frame hb;
  hb.can_id = CAN_HB_STERFBOARD;
  hb.can_dlc = 4;
  hb.data[0] = heartbeatInterval >> 8;
  hb.data[1] = heartbeatInterval & 0xFF;
  hb.data[2] = (mode == EMERGENCY) ? 0x02 : (mode == AUTO) ? 0x01 : 0x00;
  hb.data[3] = 0x00;

  mcp2515.sendMessage(&hb);

  Debug.print("CAN HB SENT id=0x");
  Debug.print(hb.can_id, HEX);
  Debug.print(" mode=");
  Debug.println(hb.data[2]);

  Serial.print("<MODE:");
  Serial.print(hb.data[2]);
  Serial.print(",HB:");
  Serial.print(hbSignal);
  Serial.print(",NOACK:");
  Serial.print(noAckReceived ? 1 : 0);
  Serial.print(",VSP:");
  Serial.print(velocitySetpoint, 2);
  Serial.print(",VFB:");
  Serial.print(velocityFeedback, 2);
  Serial.print(",SRSP:");
  Serial.print(steeringRateSetPoint, 2);
  Serial.print(",SAFB:");
  Serial.print(steeringAngleFeedback, 2);
  Serial.println(">");
}

void sendCANMessages() {
  struct can_frame steeringMsg;
  steeringMsg.can_id = CAN_STEERING_TARGET;
  steeringMsg.can_dlc = 8;
  floatToBytes(steeringAngle, steeringMsg.data);
  floatToBytes(steeringAngleVelocity, &steeringMsg.data[4]);
  mcp2515.sendMessage(&steeringMsg);

  struct can_frame velocityMsg;
  velocityMsg.can_id = CAN_VELOCITY_TARGET;
  velocityMsg.can_dlc = 8;
  floatToBytes(velocity, velocityMsg.data);
  floatToBytes(acceleration, &velocityMsg.data[4]);
  mcp2515.sendMessage(&velocityMsg);

  struct can_frame gpsMsg;
  gpsMsg.can_id = CAN_GPS_VELOCITY;
  gpsMsg.can_dlc = 4;
  floatToBytes(gpsVelocity, gpsMsg.data);
  mcp2515.sendMessage(&gpsMsg);

  Debug.print("CAN DATA SENT steer=");
  Debug.print(steeringAngle);
  Debug.print(" steerVel=");
  Debug.print(steeringAngleVelocity);
  Debug.print(" vel=");
  Debug.print(velocity);
  Debug.print(" accel=");
  Debug.print(acceleration);
  Debug.print(" gpsVel=");
  Debug.println(gpsVelocity);
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
  if (mcp2515.readMessage(&msg) != MCP2515::ERROR_OK) return;
  if (msg.can_dlc != 8) return;

  if (msg.can_id == CAN_HB_AXELBRAKE) {
    digitalWrite(canLED, HIGH);
    lastCanSig = millis();
    memcpy(&velocitySetpoint, &msg.data[0], 4);
    memcpy(&velocityFeedback, &msg.data[4], 4);

    Debug.print("RX CAN AXELBRAKE VSP=");
    Debug.print(velocitySetpoint);
    Debug.print(" VFB=");
    Debug.println(velocityFeedback);
  }
  else if (msg.can_id == CAN_HB_STEERBOK) {
    digitalWrite(canLED, HIGH);
    lastCanSig = millis();
    memcpy(&steeringRateSetPoint, &msg.data[0], 4);
    memcpy(&steeringAngleFeedback, &msg.data[4], 4);

    Debug.print("RX CAN STEERBOK SRSP=");
    Debug.print(steeringRateSetPoint);
    Debug.print(" SAFB=");
    Debug.println(steeringAngleFeedback);
  }
}

void led_maintenance() {
  nowMS = millis();
  if (nowMS - lastCanSig > canLEDDur) {
    digitalWrite(canLED, LOW);
  }
}