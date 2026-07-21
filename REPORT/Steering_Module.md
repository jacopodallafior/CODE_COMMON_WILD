In short, to achieve autonomous steering, we need:
1. an actuator to turn the wheels
2. a sensor to measure the position of the wheels
3. a local controller to communicate with the brain of the global system

The first step has been tackled previously by adding a sprocket directly on the steering column attached to a motor, but the torque was insufficient. (add picture of the chain, note that it is now used with the angle encoder)
Later a different approach was attempted, reasoning that if the motor could be attached to the steering wheel then the car's power steering would help turn, and thus a lower torque motor would suffice. A solution was designed which uses a big stepper motor which grips on the steering wheel to turn, but this was found to be slipping and cumbersome to setup and dismantle. (add picture of the old steering actuator)
Upon studying the actual working of the Jimny's steering system, we noticed that it uses a Electronic Power Steering (EPS).
There is a torque sensor on the steering column, and based on the effort input by the driver on the steering wheel a signal is sent to the EPS ECU, basically telling the motor to help steer the wheels in the direction commanded by the driver.
(add picture of original jimny steering system)
We quickly pulled out an old oscilloscope and measured what type of signals were being transmitted to check if we could easily replicate them. (add picture video of oscilloscope) 
It turns out these were simple signals, so we made a quick sniffing device with an arduino uno and tapped into the signal wires. (add schematics of the sniffer)

With these we obtained the following results: (add pictures of graphs)
Based on these we made a quick prototype board with an arduino and a DAC to check if the EPS would have enough torque to move the wheels with the car on the floor, standing still, and it succeeded.
<img width="960" height="720" alt="image" src="https://github.com/user-attachments/assets/5d07cf36-bc84-4f8c-b04c-b54ed414a557" />

Now that we have actuation, we move to step 2: obtain position of the wheels.
Previous interns adapted the chain + sprocket that was used to actuate the column to attach an encoder. This has been chosen to confirm the functionality of the system and works, but it is not an absolute encoder and because it uses a heavy chain with quite some slack, the measurement is not particularly precise, and it needs to be calibrated every time (either automatically if you let the wheels go all the way to one side and then the other, or manually by always checking they're facing straight before startup)

Finally, once everything had been tested individually, we made a board to connect to the rest of the system.
<img width="725" height="526" alt="image" src="https://github.com/user-attachments/assets/31b9d554-9afa-42c7-8c11-25ac748671e0" />



Improvement points:
- automated (safe) mode switching via relays to go between autonomous/remote control and direct control (person in the car)
- new board and permanent wiring
- better angle measurement (either better encoder or telemetry through the car's original sensor



