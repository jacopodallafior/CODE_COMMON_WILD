# Autonomous Steering Module: Engineering Log & Architecture

## System Objective
To achieve autonomous steering on the Suzuki Jimny, this module requires three integrated components:
1. **Actuator:** To physically manipulate the steering axis.
2. **Sensor:** To measure real-time wheel position.
3. **Local Controller:** To interface between the low-level hardware and the global autonomous system brain.

This document outlines the iterative development process, current system architecture, and immediate improvement points for incoming engineering teams.

---

## Phase 1: Actuation & Iterative Failures
Actuating the steering mechanically proved difficult. The first two attempts failed due to mechanical constraints, leading us to our current electrical solution. 

*   **Iteration 1: Direct Column Drive** 
    Previous interns attached a motor-driven sprocket directly to the steering column via a chain. 
    *Result:* Failed. The motor output torque was insufficient to turn the column under load. 
    <img width="1600" height="1200" alt="steering_first" src="https://github.com/user-attachments/assets/426eec7c-3a9a-4897-bc44-52faad88e954" />
    *(Note: This chain mechanism was later repurposed for the angle encoder, detailed in Phase 3).*

*   **Iteration 2: Steering Wheel Friction Drive** 
    After, they thought they could leverage the vehicle's native power steering, and mounted a large stepper motor directly to the steering wheel to drive it via friction.
    *Result:* Failed. The mechanism suffered from excessive slip and the mounting hardware was cumbersome to install and dismantle.
    <img width="4032" height="3024" alt="IMG_2872" src="https://github.com/user-attachments/assets/8fccff29-8457-44ca-a8f4-faac90ec03bd" />

---

## Phase 2: The EPS Breakthrough
Instead of fighting the steering mechanics, we investigated the Jimny's native Electronic Power Steering (EPS). 

The EPS system uses a torque sensor on the steering column. When a driver turns the wheel, the sensor sends a proportional signal to the EPS ECU, which commands an assist motor. **Our solution: Spoof the torque sensor signal to make the EPS motor do the work for us.**

<img width="1200" height="1600" alt="steering_column" src="https://github.com/user-attachments/assets/2bb007c5-97d9-4b62-adb9-5af22516033c" />
*Jimny native steering column and EPS architecture.*

### Signal Interception
We probed the native EPS torque sensor lines using an oscilloscope to characterize the signaling protocol. 

<p float="left">
  <img width="500" alt="oscilloscope_connection" src="https://github.com/user-attachments/assets/392d5477-dd01-403e-b1c2-a0ab73cbae60" />
  <img width="500" alt="oscilloscope" src="https://github.com/user-attachments/assets/15153a2e-49b3-4283-9411-d3f8a3166665" />
</p>

The signals were straightforward analog voltages. We built a custom sniffer board using an Arduino Uno to tap into the signal wires and map the required voltage profiles for left and right actuation.

<img width="1274" height="879" alt="image (1)" src="https://github.com/user-attachments/assets/b05060fe-f27b-40d5-96e2-fec1ce80239e" />
*Arduino-based signal sniffer schematic.*

**Captured Signal Profiles:**
<p float="left">
  <img width="309" height="153" alt="graph_steering1" src="https://github.com/user-attachments/assets/d4948784-f74e-4e5e-959e-f30e8a972925" />
  <img width="253" height="192" alt="graph_steering2" src="https://github.com/user-attachments/assets/af32b5bd-db94-4659-8c70-e3b8deb46180" />
  <img width="360" height="264" alt="graph_steering3" src="https://github.com/user-attachments/assets/562874cb-6f0e-4081-ae93-fbfeee04402d" />
</p>

### EPS Actuation Proof of Concept
We built a prototype board combining an Arduino and a Digital-to-Analog Converter (DAC) to inject simulated voltage signals directly into the EPS ECU. 
*Result:* Success. The injected signals commanded the EPS to output enough torque to actuate the wheels while the vehicle was stationary on the ground (static dry-steering).

<img width="960" height="720" alt="image" src="https://github.com/user-attachments/assets/5d07cf36-bc84-4f8c-b04c-b54ed414a557" />
*DAC Injection Prototype.*

---

## Phase 3: Position Sensing
With actuation solved, we required real-time steering angle telemetry. 

We retrofitted the discarded chain-and-sprocket hardware from Iteration 1 to drive a rotary encoder. While this confirmed the system logic works, it introduces two major structural flaws that must be addressed in future iterations:
1. **Mechanical Hysteresis:** The heavy chain introduces significant slack, severely degrading measurement precision.
2. **Relative Measurement:** The current encoder is not absolute. The system requires manual calibration at every boot (either physically aligning the wheels dead-center, or running an automated sweep from lock-to-lock).

---

## Phase 4: System Integration
Following individual component validation, we designed and assembled the final local control board. This board integrates the DAC signal injection and the encoder telemetry, serving as the bridge to the global system brain.

<p float="left">
  <img width="500" alt="image" src="https://github.com/user-attachments/assets/31b9d554-9afa-42c7-8c11-25ac748671e0" />
  <img width="500" alt="steering_finished" src="https://github.com/user-attachments/assets/ede3d55d-4942-469e-a7e0-5130e53b9227" />
</p>
*Final integrated board design and physical installation.*

---

## Phase 5: Next Steps & Roadmap
Immediate priorities for incoming interns center on safety, precision, and robustness:

*   **Mode-Switching Interlocks:** Implement automated, relay-driven mode switching. The system requires a fault-tolerant method to toggle between autonomous control, remote control, and direct manual override (driver in the cabin).
*   **Sensor Upgrade:** Eliminate the chain-drive entirely. Source and mount an absolute rotary encoder directly to the column, or reverse-engineer the vehicle's native CAN bus to extract existing steering telemetry.
*   **Hardware Hardening:** Replace prototype wiring with a permanent, vibration-resistant PCB and shielded wiring harness.
