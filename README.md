# rc_car_throttle_limiter
Arduino based throttle limiter for RC cars. Reduce speed for kits, but with full break control.

// by Marko Hoepken
// 25th Dec 2015
// RC Car throttle limiter.
//
// This simple Arduino application will go into an RC Car.
// The Arduino will sit between the receiver and the ESC controlling the motor.
// 
// The "kits" poblem: Most ready to run cars do not have a computer transmitter.
// This means it is not possible to limit the maximum throttle (= max speed).
// A throttle limit will prevent too fast running for beginners (my younger son ;-)).
// 
// This application does the following:

// 1. Provides a single key programming of maximum speed
// 2. Forward is limited to the maximum speed.
// 3. Reverse is limited to the maximum speed (scalled from forward speed)
// 4. A simple state machine provides full BREAK (= full revers), but limits backward speed

// Note to 4.
// My ESC does has the follwing common behaviour:
// From forward to backward = BREAK only. Max reverse = Maximum break.
// For backward, it is required to go from Break to neutral, than back again

