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
/*


0. For resetting to factory  values, PRESS the key while turning on, untile the LED will flash fast

After regular start:
1. Enter programming by pressing the key >3 seconds.
   The green LED will get ON. Release key
2. State: Prgrogramm neutral 
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
   */

#ifndef TXLED0 // Use inbuilt led (pin 13) on Arduino Nano boards instead of TXLED
  #ifdef ARDUINO_AVR_NANO
    #define TXLED0      PORTB &= ~(1<<5)
    #define TXLED1      PORTB |= (1<<5)
  #endif
#endif

#define SERIAL_DEBUG 0

// Pining.
#define PROG_KEY 2 // push button to GND
#define RC_IN 5    // connect to receiver channel 1
#define RC_OUT 6   // connect to ESC
#define LED 13     // show state

// PPM values
#define STOP_RANGE 30 // size of neutral zone, deppends on jitter of TX
                      // too small value = bad detection of break
                      // too large value = large dead band
                      // on programming there is a calibration for that
#define STOP_RANGE_SPARE 5 // will be added to range of measured rx jitter                      
#define STOP 1500     // average neutral zone
#define MAX 2000      // max value
#define MIN 1000      // bin value

// state coding operations
#define STATE_STOP 0     // neutal zone of Rx signal
#define STATE_FORWARD 1  // forward driving
#define STATE_BACKWARD 2 // backward driving
#define STATE_BREAK 3    // breaking

#define ENTER_DELAY 20
#define DEBOUNCE_DELAY 100
uint8_t key_pressed=0;

#define HEARD_BEAT_LOOP 150
uint8_t heard_beat =0;

// eeprom handling
#define EEPROM_ADR_MIN_L 0
#define EEPROM_ADR_MIN_H 1
#define EEPROM_ADR_MAX_L 2
#define EEPROM_ADR_MAX_H 3
#define EEPROM_ADR_STOP_L 4
#define EEPROM_ADR_STOP_H 5
#define EEPROM_ADR_RANGE_L 6
#define EEPROM_ADR_RANGE_H 7
#define EEPROM_ADR_MAGIC_KEY 8
#define EEPROM_MAGIC_KEY_SIZE 17
const uint8_t MagicKey[] PROGMEM = { // key to check if EEprom matches software
  'T','H','R','O','T','T','L','E',' ',
  'R','E','V','1','.','0','.','1'
};

#include <EEPROM.h>
#include <Servo.h> 
Servo my_esc;  // create servo object to control a servo 
 
// range limits 
uint16_t rc_out_max = MAX;
uint16_t rc_out_min = MIN;
uint16_t rc_in_neutral = STOP;
uint16_t rc_neutral_range = STOP_RANGE;
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
    // RESET VALUE HANDLER
    uint8_t eeprom_invalid=0;
    while(!digitalRead(PROG_KEY))
    {
        flash_led(1, 50, 50);
        eeprom_invalid=1; // force reset of values
    }  
    if(eeprom_invalid==1)
    {
        // INDICATE RESET
        flash_led(5, 100, 255);
    }
    // check if eeprom is matching current software
    // the check is done by comparing magic key in eeprom
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
        // save 16 bit        
        EEPROM.write(EEPROM_ADR_RANGE_L,lowByte(STOP_RANGE));
        EEPROM.write(EEPROM_ADR_RANGE_H,highByte(STOP_RANGE));        
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
    rc_neutral_range=((EEPROM.read(EEPROM_ADR_RANGE_H)<<8) | (EEPROM.read(EEPROM_ADR_RANGE_L)));    
    rc_out=rc_in_neutral;
} 
 
 
void loop() 
{ 
    // PROGRAMMING MODE
    // allow only after 5 seconds after start, to prevent changes
    // by accident
    
    // programming statemachine
    
    // 1. enter programming mode by pressing the key ~3 seconds
    if(!digitalRead(PROG_KEY)) // key pressed
    {
        key_pressed++;   
        delay(DEBOUNCE_DELAY);
        if(key_pressed > ENTER_DELAY)
        {
            uint8_t not_saved=1;
            // 1. ENTER
            // flash to show endered mode
            // flash until release
            while(!digitalRead(PROG_KEY))
            {
                TXLED1;
            }  
            TXLED0;
            delay(500);            
            // show next stage
            while(digitalRead(PROG_KEY))
            {
                flash_led(1, 10, 50);
                (void)wait_and_drive(1000,0);
            }
            // 2. Save Neutral
            while(!digitalRead(PROG_KEY))
            {
                TXLED1;
                if(not_saved)
                {
                    // prog mode, reset limits
                    rc_out_max=MAX;
                    rc_out_min=MIN;  
                    // set neutral position
                    rc_in_neutral=read_rc(20);                    
                    rc_neutral_range=read_rc_jitter (20);
                    not_saved=0;
                }
            } 
            TXLED0;
            delay(500);
            // show next stage
            // 3. MAX
            // measure max value
            rc_out_max=0;
            while(digitalRead(PROG_KEY))
            {                
                flash_led(2, 10, 50);
                uint16_t max=wait_and_drive(1000,1);
                if(rc_out_max<max) // keep highest
                {
                    rc_out_max=max;
                }
            }        
            while(!digitalRead(PROG_KEY))
            {
                TXLED1;
            }  
            TXLED0;
            delay(500);
            // show next stage
            // 4. MIN
            // measure min value
            rc_out_min=2500;
            while(digitalRead(PROG_KEY))
            {
                flash_led(3, 10, 50);
                uint16_t min=wait_and_drive(1000,0);
                if(rc_out_min>min) // keep lowest
                {
                    rc_out_min=min;
                }
            }        
            while(!digitalRead(PROG_KEY))
            {
                TXLED1;
            }
            TXLED0;            
            delay(1000);       
            // done
            flash_led(5, 100, 255);
            // save 16 bit values to eeprom
            EEPROM.write(EEPROM_ADR_MIN_L,(rc_out_min & 0xff));        
            EEPROM.write(EEPROM_ADR_MIN_H,(rc_out_min >> 8));    
            EEPROM.write(EEPROM_ADR_MAX_L,(rc_out_max & 0xff));        
            EEPROM.write(EEPROM_ADR_MAX_H,(rc_out_max >> 8));    
            EEPROM.write(EEPROM_ADR_STOP_L,(rc_in_neutral & 0xff));        
            EEPROM.write(EEPROM_ADR_STOP_H,(rc_in_neutral >> 8));    
            EEPROM.write(EEPROM_ADR_RANGE_L,(rc_neutral_range & 0xff));        
            EEPROM.write(EEPROM_ADR_RANGE_H,(rc_neutral_range >> 8));  
            
        }
    }
    else // run mode
    {
        // heard beat
        TXLED0;
        if(heard_beat++>HEARD_BEAT_LOOP)
        {
            heard_beat=0;
            TXLED1;
        }
        key_pressed=0;  // reset counter for programming mode enter

      
        // NORMAL OPERATION
        // read input value from receiver
        rc_in = pulseIn(RC_IN, HIGH, 25000); // Read the pulse width of input
          
        // FORWARD CASE
        if(rc_in > (rc_in_neutral+ rc_neutral_range))
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
            rc_in < (rc_in_neutral+ rc_neutral_range)
            &&
            rc_in > (rc_in_neutral - rc_neutral_range)
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
        if(rc_in < (rc_in_neutral- rc_neutral_range))
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
    }    
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
    Serial.print(rc_out);        // new output
    Serial.print(" Jitter; "); // Print the value of 
    Serial.println(rc_neutral_range);        // new output
    
#endif 
        
} 

// simple helper function to create a number of LED flashes
void flash_led(uint8_t count, uint8_t on, uint8_t off)
{
    for(uint8_t loop=0; loop< count; loop++)
    {
        TXLED1;
        delay(on);
        TXLED0;
        delay(off);
    }
}

// read count times the RC input an return average
uint16_t read_rc (uint8_t count)
{
    uint32_t input=0;
    pulseIn(RC_IN, HIGH, 25000);
    for(uint8_t loop=0; loop< count; loop++)
    {
        input+=pulseIn(RC_IN, HIGH, 25000);
    }
    return(input/count); 
}

// measures jitter for good neutral detection
uint16_t read_rc_jitter (uint8_t count)
{
    uint32_t input=0;
    uint32_t min=2500;
    uint32_t max=0;
    for(uint8_t loop=0; loop< count; loop++)
    {
        input=pulseIn(RC_IN, HIGH, 25000);
        if(input<min)
        {
            min=input;
        }
        if(input>max)
        {
            max=input;
        }
    }
    // calualte jitter
    return(max-min+STOP_RANGE_SPARE);
}

// to a long wait spitted into 20 shorts and upate rc output during wait
// can be used to wait long, bur Rc controll still works
// mode stores min or max value while waiting
// mode 0 = min
// mode 1 = max
uint16_t wait_and_drive(uint16_t wait, uint8_t mode)
{
   uint8_t count=20; 
   uint16_t value =2500; // init for min
   if(mode) // set to 0 on max check
   {
        value=0;
   }
   uint16_t rc_in=0;
   for(uint8_t loop=0; loop< count; loop++)
    {
        rc_in=pulseIn(RC_IN, HIGH, 25000);
        if(mode) // max
        {
            if(rc_in>value)
            {
                value=rc_in;
            }
        }
        else // min
        {
            if(rc_in<value)
            {
                value=rc_in;
            }        
        }
        my_esc.writeMicroseconds(rc_in);
        delay(wait/count); // to partial wait
    }        
    return(value);
}
