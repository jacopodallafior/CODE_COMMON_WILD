#pragma once

// ── Heartbeats ─────────────────────────────────────────────────
#define CAN_HB_STERFBOARD    0x01   // Sterfboard → all
#define CAN_HB_STEERBOK      0x11   // Steerbok → all
#define CAN_HB_AXELBRAKE     0x12   // AxelBrake → all
#define CAN_HB_BRAKE         0x13   // BrakeStepper → all  
#define CAN_HB_STEERING      0x14

// ── Commands ───────────────────────────────────────────────────
#define CAN_STEERING_TARGET  0x110  // Sterfboard → Steerbok
#define CAN_VELOCITY_TARGET  0x120  // Sterfboard → AxelBrake
#define CAN_GPS_VELOCITY     0x121  // Sterfboard → all
#define CAN_BRAKE_PCT        0x130  // AxelBrake  → BrakeStepper