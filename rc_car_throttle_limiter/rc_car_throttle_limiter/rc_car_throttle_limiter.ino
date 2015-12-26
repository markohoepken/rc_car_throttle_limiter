// RC Car throttle limiter.
// by Marko Hoepken
// 25th Dec 2015
//
// This simple Arduino application will go into an RC Car.
// The Arduino will sit between the receiver and the ESC controlling the motor.
// 
// The "kids" poblem: Most ready to run cars do not have a computer transmitter.
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

// Programming mode for range:
//
// SAFETY NOTE: While programming, make sure, that wheels are free in the air!
// KEEY YOUR FINGERS SAVE!
// WEAR SAFTY GLASSED TO PREVENT THINGS FLY IN YOU EYES
//
// 1. Programming MUST be done within 5 seconds after start
// 2. Put tranceiver to neutral
// 3. Press key AND HOLD. This saves the neutral position.
// 4. Now you can give as much "throttle" (FORWARD) as you want for max power.
// 5. Release the key. Curent max value is saved and also take for min


#define SERIAL_DEBUG 0

// Pining.
#define PROG_KEY 2 // push button to GND
#define RC_IN 5    // connect to receiver channel 1
#define RC_OUT 6   // connect to ESC
#define LED 13     // show state

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

// eeprom handling
#define EEPROM_ADR_MIN_L 0
#define EEPROM_ADR_MIN_H 1
#define EEPROM_ADR_MAX_L 2
#define EEPROM_ADR_MAX_H 3
#define EEPROM_ADR_STOP_L 4
#define EEPROM_ADR_STOP_H 5
#define EEPROM_ADR_MAGIC_KEY 6
#define EEPROM_MAGIC_KEY_SIZE 17
const uint8_t MagicKey[] PROGMEM = { // key to check if EEprom matches software
  'T','H','R','O','T','T','L','E',' ',
  'R','E','V','1','.','0','.','0'
};

#include <EEPROM.h>
#include <Servo.h> 
Servo my_esc;  // create servo object to control a servo 
 
// range limits 
uint16_t rc_out_max = MAX;
uint16_t rc_out_min = MIN;
uint16_t rc_in_neutral = STOP;
// init states
uint8_t state=STATE_STOP;
uint8_t last_state=STATE_STOP; 
// in and out values
uint16_t rc_in=0;
uint16_t rc_out=rc_in_neutral;
// just to disable programming after 10 seconds for security 
uint8_t enable_prog=1;
 
void setup() 
{ 

  // init servo
  my_esc.attach(RC_OUT);  // attaches the servo on pin 9 to the servo object 
  my_esc.writeMicroseconds(rc_in_neutral);
  // init system
  state=STATE_STOP;
  last_state=state;
  // input key for programming
  pinMode(PROG_KEY, INPUT_PULLUP);   
  // check if eeprom is matching current software
  // the check is done by comparing magic key in eeprom
  uint8_t eeprom_invalid=0;
  for(int i=0; i<EEPROM_MAGIC_KEY_SIZE; i++)
   {
        if((uint8_t)EEPROM.read(EEPROM_ADR_MAGIC_KEY+i) != (uint8_t)pgm_read_word_near(MagicKey + i))
        {
            eeprom_invalid=1;
            break;
        }        
   }
    if(eeprom_invalid) // EEprom does not match current software, must store new defaults
    {
        // save 16 bit
        EEPROM.write(EEPROM_ADR_MIN_L,lowByte(MIN));        
        EEPROM.write(EEPROM_ADR_MIN_H,highByte(MIN));    
        // save 16 bit
        EEPROM.write(EEPROM_ADR_MAX_L,lowByte(MAX));
        EEPROM.write(EEPROM_ADR_MAX_H,highByte(MAX));
        // save 16 bit
        EEPROM.write(EEPROM_ADR_STOP_L,lowByte(STOP));
        EEPROM.write(EEPROM_ADR_STOP_H,highByte(STOP));
        // store magic key for next check
        for(int i=0; i<EEPROM_MAGIC_KEY_SIZE; i++)
        {
            EEPROM.write(EEPROM_ADR_MAGIC_KEY+i,pgm_read_word_near(MagicKey + i));
        }         
    }
    // read settings from EEProm
    rc_out_min=((EEPROM.read(EEPROM_ADR_MIN_H)<<8) | (EEPROM.read(EEPROM_ADR_MIN_L)));
    rc_out_max=((EEPROM.read(EEPROM_ADR_MAX_H)<<8) | (EEPROM.read(EEPROM_ADR_MAX_L)));
    rc_in_neutral=((EEPROM.read(EEPROM_ADR_STOP_H)<<8) | (EEPROM.read(EEPROM_ADR_STOP_L)));
    rc_out=rc_in_neutral;
} 
 
 
void loop() 
{ 
  // PROGRAMMING MODE
  // allow only after 5 seconds after start, to prevent changes
  // by accident
  if(  enable_prog && ((millis() > 5000)))
  {
    enable_prog=0; // disable programming
  }
  // check if key is pressed, in that case go to prog mode
  if(enable_prog && !digitalRead(PROG_KEY))
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
    // save 16 bit values to eeprom
    EEPROM.write(EEPROM_ADR_MIN_L,(rc_out_min & 0xff));        
    EEPROM.write(EEPROM_ADR_MIN_H,(rc_out_min >> 8));    
    EEPROM.write(EEPROM_ADR_MAX_L,(rc_out_max & 0xff));        
    EEPROM.write(EEPROM_ADR_MAX_H,(rc_out_max >> 8));    
    EEPROM.write(EEPROM_ADR_STOP_L,(rc_in_neutral & 0xff));        
    EEPROM.write(EEPROM_ADR_STOP_H,(rc_in_neutral >> 8));            
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
    
 #if SERIAL_DEBUG == 1

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
#endif 
        
} 