# JIMNY STEERING SYSTEM  – Arduino + MCP4728 + Encoder

## Overview

This repository contains three Arduino sketches used to control and test a steering actuation system based on:

* Quadrature encoder feedback (angle measurement)
* MCP4728 DAC (analog output generation)
* Differential voltage control (channel A / channel B)
* Optional PID closed-loop control

The files are organized by increasing system complexity:

1. `writeDACplusread.ino` → Basic DAC test with serial logging
2. `readDACtoCAR.ino` → DAC control driven externally (e.g., Python)
3. `PIDfirst.ino` → Full closed-loop control with encoder + PID

---

## 1. writeDACplusread.ino

### Purpose

This is a **low-level DAC test and debugging tool**.
It allows you to:

* Send a voltage delta via serial
* Apply it symmetrically to two DAC channels
* Monitor outputs in real time

### Key Features

* Converts voltage → DAC code (12-bit resolution)

* Applies **differential control**:

  ```
  VA = IDLE_A + delta
  VB = IDLE_B - delta
  ```

* Outputs structured serial data:

  ```
  DATA,t_ms,delta,va,vb,codeA,codeB
  ```

### Use Case

* Validate wiring of MCP4728
* Verify voltage scaling and calibration
* Check system response before closing the loop

### Serial Commands

* Send a number → sets `delta` (in volts)
* `RESET` → returns to idle condition

---

## 2. readDACtoCAR.ino

### Purpose

This is a **clean interface for external control**, typically from Python or another host system.

Compared to the previous file:

* Less logging noise
* Focused on command-response behavior
* Suitable for integration with higher-level controllers

### Key Features

* Same differential DAC logic
* Tighter delta limit (safety constraint)
* Clean serial output:

  ```
  DELTA,<val>,VA,<val>,VB,<val>,CODEA,<val>,CODEB,<val>
  ```

### Control Model

External system (e.g., Python) computes control input:

```
delta = f(controller)
```

Arduino only executes:

```
applyDelta(delta)
```

### Use Case

* Hardware-in-the-loop testing
* Rapid prototyping of controllers in Python
* System identification experiments

---

## 3. PIDfirst.ino

### Purpose

This is the **full embedded closed-loop controller**.

It integrates:

* Encoder feedback
* Angle estimation
* PID control
* DAC actuation

This is the core of your steering system.

---

### System Architecture

```
Encoder → Angle → PID → Delta Voltage → DAC → Actuator
```

---

### Encoder Subsystem

* Quadrature encoder using interrupt-based decoding
* High-resolution counting
* Direction detection via state transitions

Angle computation:

```
angle_deg = 360 * (counts / counts_per_rev) / gear_ratio
```

Includes:

* ISR event tracking
* Invalid transition detection (noise/debugging)

---

### DAC Control

Same principle as other files:

```
VA = IDLE_A + delta
VB = IDLE_B - delta
```

With:

* Saturation (`DELTA_LIMIT`)
* Voltage clipping
* Calibration offsets (`CAL_A`, `CAL_B`)

---

### PID Controller

Discrete-time controller running every:

```
10 ms (100 Hz)
```

Control law:

```
u = Kp * error + Ki * integral - Kd * d(measurement)/dt
```

Important design choices:

* Derivative on measurement (noise reduction)
* Integral windup protection
* Output saturation

---

### Serial Interface

#### Control Commands

* `e` → Enable PID
* `d` → Disable PID
* `z` → Reset encoder and state
* `r` → Reverse encoder direction
* `p` → Print current data

#### Setpoints and Gains

* `s<value>` → target angle (deg)
* `kp<value>` → proportional gain
* `ki<value>` → integral gain
* `kd<value>` → derivative gain

---

### Data Logging

Structured output for analysis:

```
DATA,t_ms,target,angle,error,delta,va,vb,codeA,codeB,count,pid_on,p,i,d,isr,invalid
```

This is extremely useful for:

* Plotting responses
* Tuning PID
* Debugging instability

---

## Key Design Concepts

### 1. Differential Voltage Control

Instead of driving a single signal:

* Two channels are used in opposition
* Improves symmetry and control resolution

---

### 2. Open-loop vs Closed-loop

| Mode             | File             |
| ---------------- | ---------------- |
| Open-loop DAC    | writeDACplusread |
| External control | readDACtoCAR     |
| Embedded control | PIDfirst         |

---

### 3. Safety Constraints

All files enforce:

* Voltage limits
* Delta saturation
* Safe startup (idle output)

---

## Recommended Workflow

1. Start with `writeDACplusread.ino`

   * Validate hardware and DAC behavior

2. Move to `readDACtoCAR.ino`

   * Test control from Python / PC

3. Deploy `PIDfirst.ino`

   * Run fully embedded control

Skipping this order is how people end up chasing ghosts in their system.

---

## Final Notes

* Always measure **real DAC VDD** and update constants
* Tune PID gains gradually (start with Kp only)
* Watch for encoder noise (invalid transitions counter helps)
* Log everything — your future self will thank you

---

If you want, I can also:

* Help you tune the PID step-by-step
* Build a Python script for plotting/logging
* Or refactor this into a cleaner modular architecture

Just say the word.
