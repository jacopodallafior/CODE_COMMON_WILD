In short, to achieve autonomous steering, we need:
1. an actuator to turn the wheels
2. a sensor to measure the position of the wheels
3. a local controller to communicate with the brain of the global system
The first step has been tackled previously by adding a sprocket directly on the steering column attached to a motor, but the torque was insufficient. (add picture of the chain, note that it is now used with the angle encoder)
Later a different approach was attempted, reasoning that if the motor could be attached to the steering wheel then the car's power steering would help turn, and thus a lower torque motor would suffice. A solution was designed which uses a big stepper motor which grips on the steering wheel to turn, but this was found to be slipping and cumbersome to setup and dismantle. (add picture of the old steering actuator)
Upon studying the actual working of the Jimny's steering system, we noticed that it uses a Electronic Power Steering (EPS).
There is a torque sensor on the steering column, and based on the effort input by the driver on the steering wheel a signal is sent to the EPS ECU, basically telling the motor to help steer the wheels in the direction commanded by the driver.
(add picture of original jimny steering system)
We quickly pulled out an old oscilloscope and measured what type of signals were being transmitted to check if we could easily replicate them. It turns out these were simple signals
