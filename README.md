# rc_car_throttle_limiter
Arduino based throttle limiter for RC cars. Reduce speed for kids, but with full break control.

CHECK THE WIKI FOR SCHEMATICS AND SOME PICTURES.

by Marko Hoepken... 5 hour project for christmas gift ;-)
25th Dec 2015
RC Car throttle limiter.

This simple Arduino application will go into an RC Car.
The Arduino will sit between the receiver and the ESC controlling the motor.

The "kids" problem: Most ready to run cars do not have a computer transmitter.
This means it is not possible to limit the maximum throttle (= max speed).
A throttle limit will prevent too fast running for beginners (my younger son ;-)).

This application does the following:

1. Provides a single key programming of maximum speed
2. Forward is limited to the maximum speed.
3. Reverse is limited to the maximum speed (scalled from forward speed)
4. A simple state machine provides full BREAK (= full revers), but limits backward speed

Note to 4.
My ESC does has the follwing common behaviour:
From forward to backward = BREAK only. Max reverse = Maximum break.
For backward, it is required to go from Break to neutral, than back again

Programming mode for range:

SAFETY NOTE: While programming, make sure, that wheels are free in the air!
KEEY YOUR FINGERS SAVE!
WEAR SAFTY GLASSED TO PREVENT THINGS FLY IN YOU EYS

0. For resetting to factory  values, PRESS the key while turning on, untile the LED will flash fast

After regular start:
1. Enter programming by pressing the key >3 seconds.

   The green LED will get ON. Release key
   
2. State: Prgrogramm neutral 
3.
   This will be indicated by 1 LED flash with pause.

   Set RX to neutral. 

   Press KEY until LED will become permanent ON. 
   
   Release Key
   
3. State: Prgrogramm MAX

   This will be indicated by 2 LED flashes with pause.
   
   Set RX to wanted maximum value, and HOLD.
   
   Press KEY until LED will become permanent ON. 
   
   Release Key
   
3. State: Prgrogramm MIN

   This will be indicated by 3 LED flashes with pause.
   
   Set RX to wanted minimum value, and HOLD.
   
   Press KEY until LED will become permanent ON. 
   
   Release Key
   
4. Settings will be stored.

   This will be indicated by 5 LED flashes with pause.
   
