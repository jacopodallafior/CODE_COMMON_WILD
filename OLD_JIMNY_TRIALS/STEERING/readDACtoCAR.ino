#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// Misura il VCC reale del DAC col multimetro tra VCC e GND.
// Se leggi 4.97 V, scrivi 4.97 qui.
const float DAC_VDD = 5.15;
const int DAC_MAX = 4095;

// Idle fissi del banco
const float IDLE_A = 2.4400;
const float IDLE_B = 2.4200;

// Limite massimo di spostamento rispetto all'idle
const float DELTA_LIMIT = 0.25;

// Correzione fine per canale, se serve
float CAL_A = 0.0000;
float CAL_B = 0.0000;

// Delta attuale
float delta_now = 0.0;

uint16_t voltToCode(float v) {
  if (v < 0.0) v = 0.0;
  if (v > DAC_VDD) v = DAC_VDD;
  return (uint16_t)((v / DAC_VDD) * DAC_MAX + 0.5);
}

void applyDelta(float delta) {
  if (delta > DELTA_LIMIT) delta = DELTA_LIMIT;
  if (delta < -DELTA_LIMIT) delta = -DELTA_LIMIT;

  delta_now = delta;

  float outA = IDLE_A + delta_now + CAL_A;
  float outB = IDLE_B - delta_now + CAL_B;

  if (outA < 0.0) outA = 0.0;
  if (outA > DAC_VDD) outA = DAC_VDD;

  if (outB < 0.0) outB = 0.0;
  if (outB > DAC_VDD) outB = DAC_VDD;

  uint16_t codeA = voltToCode(outA);
  uint16_t codeB = voltToCode(outB);

  mcp.setChannelValue(MCP4728_CHANNEL_A, codeA);
  mcp.setChannelValue(MCP4728_CHANNEL_B, codeB);

  Serial.print("DELTA,");
  Serial.print(delta_now, 4);
  Serial.print(",VA,");
  Serial.print(outA, 4);
  Serial.print(",VB,");
  Serial.print(outB, 4);
  Serial.print(",CODEA,");
  Serial.print(codeA);
  Serial.print(",CODEB,");
  Serial.println(codeB);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (!mcp.begin()) {
    Serial.println("MCP4728_NOT_FOUND");
    while (1) {}
  }

  // All'avvio vai a idle
  applyDelta(0.0);
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.length() == 0) return;

    if (s == "RESET") {
      applyDelta(0.0);
      return;
    }

    // Python manda solo un numero: il delta in volt
    float newDelta = s.toFloat();
    applyDelta(newDelta);
  }
}