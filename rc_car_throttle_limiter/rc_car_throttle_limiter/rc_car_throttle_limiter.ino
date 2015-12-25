// Sweep
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

// Pining.
#define PROG_KEY 2 // push button to GND
#define RC_IN 5    // connect to receiver channel 1
#define RC_OUT 6   // connect to ESC

// PPM values
#define STOP_RANGE 40 // size of neutral zone 
#define STOP 1500     // average neutral zone
#define MAX 2000      // max value
#define MIN 1000      // bin value

// state coding
#define STATE_STOP 0     // neutal zone of Rx signal
#define STATE_FORWARD 1  // forward driving
#define STATE_BACKWARD 2 // backward driving
#define STATE_BREAK 3    // breaking

#include <Servo.h> 
Servo my_esc;  // create servo object to control a servo 
 
// range limits 
uint16_t rc_out_max = 1750;
uint16_t rc_out_min = 1200;
uint16_t rc_in_neutral = STOP;
// init states
uint8_t state=STATE_STOP;
uint8_t last_state=STATE_STOP; 
// in and out values
uint16_t rc_in=0;
uint16_t rc_out=rc_in_neutral;
 
 
void setup() 
{ 
  my_esc.attach(RC_OUT);  // attaches the servo on pin 9 to the servo object 
  state=STATE_STOP;
  last_state=state;
  my_esc.writeMicroseconds(rc_in_neutral);
  // input key for programming
  pinMode(PROG_KEY, INPUT_PULLUP);  
} 
 
 
void loop() 
{ 
  // PROGRAMMING MODE
  // check if key is pressed, in that case go to prog mode
  if(!digitalRead(PROG_KEY))
  {
    // prog mode, reset limits
    rc_out_max=MAX;
    rc_out_min=MIN;  
    // set neutral position
    rc_in_neutral=pulseIn(RC_IN, HIGH, 25000); // Read the pulse width of input    
    // loop until key release to capture new values
    while(!digitalRead(PROG_KEY))
    {
        rc_out = pulseIn(RC_IN, HIGH, 25000); // Read the pulse width of input
        my_esc.writeMicroseconds(rc_out);
    }
    //  calculate new limits
    rc_out_max=rc_out;
    //  min is calculated from MAX
    rc_out_min=rc_in_neutral-(rc_out-rc_in_neutral);
  }

  // NORMAL OPERATION
  // read input value from receiver
  rc_in = pulseIn(RC_IN, HIGH, 25000); // Read the pulse width of input
    
  // FORWARD CASE
  if(rc_in > (rc_in_neutral+ STOP_RANGE))
  {
    state= STATE_FORWARD;
    // do output calculation
    // linar interpolation to scale wanted maximum value to output on forward
    float calc_out = rc_in_neutral + (float)(rc_in-rc_in_neutral)*((float)(rc_out_max-rc_in_neutral)/(float)(MAX-rc_in_neutral));
    // set output
    rc_out=calc_out;
  }  
  // STOP CASE
  // keep track of last state to have correct break management
  else if (
    // neutral zone
    rc_in < (rc_in_neutral+ STOP_RANGE)
    &&
    rc_in > (rc_in_neutral - STOP_RANGE)
  )
  {
    if(state != STATE_STOP) // State change
    {
        last_state=state; // forward, backward or break
    }
    state= STATE_STOP;
    rc_out=rc_in_neutral;
  }
  // BREAK / REVERSE CASE
  // break / reverse state machine
  if(rc_in < (rc_in_neutral- STOP_RANGE))
  {
    if(
        (state != STATE_BREAK)
        &&
        (state != STATE_BACKWARD)    
    ) // change of state
    {
        if(last_state== STATE_FORWARD) // last was backward, must be break
        {
            state= STATE_BREAK;
        }
        else  // othercase must be reverse
        {
            state= STATE_BACKWARD;
        }
    }        
    // break handler
    if(state== STATE_BREAK)
    {
        // break, pass full value
        rc_out=rc_in;
    }
    // backward handler
    if(state== STATE_BACKWARD)
    {
        // reverse handler
        // do output calculation
        // linar interpolation to scale wanted minimum value to output on reverse      
        float calc_out = rc_in_neutral - (float)(rc_in_neutral-rc_in)*((float)(rc_in_neutral-rc_out_min)/(float)(rc_in_neutral-MIN)); 
        // set output        
        rc_out=calc_out;
    }
  }
  // put output to servo
  my_esc.writeMicroseconds(rc_out);
    
  

  switch(state)
  {
    case STATE_STOP: Serial.print("STOP ");
    break;
    case STATE_FORWARD: Serial.print("FORWARD ");
    break;
    case STATE_BACKWARD: Serial.print("BACKWARD ");
    break;    
    case STATE_BREAK: Serial.print("BREAK ");
    break;    
  }
  switch(last_state)
  {
    case STATE_STOP: Serial.print("(STOP) ");
    break;
    case STATE_FORWARD: Serial.print("(FORWARD) ");
    break;
    case STATE_BACKWARD: Serial.print("(BACKWARD) ");
    break;    
    case STATE_BREAK: Serial.print("(BREAK) ");
    break;    
  }  
  Serial.print("throttle:"); // Print the value of 
  Serial.print(rc_in);        // current input
  Serial.print(" => "); // Print the value of 
  Serial.println(rc_out);        // new output
  
  
  
  
      
} 