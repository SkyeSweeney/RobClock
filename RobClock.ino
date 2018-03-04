//*********************************************************************
// Project: Rob's Retierement Clock
//
// Abstract: This clock displays time by means of two analog voltage
// meters. The needle on the left meter displays hours and the one
// on the left display minutes. The clock also has two buttons for
// incrementing and decrementing the time to set it. Inside the clock
// is a Sparkfun Arduino Micro Pro that runs this code as well as a
// DS3234 Real Time Clock (RTC) module also made by Sparkfun. The RTC
// talks to the Arduino using an SPI bus (Clk, MISO, MOSI, CS).
//
// The code for the RTS came directly from Sparkfun. The code for the
// Arduino (this code) was custom written.
//
// This code performs three major functions. 
// 1) On a periodic basis it reads the RTC and updates the needles. 
// 2) It monitor the two switches and modifies the RTC's time accordingly.
// 3) It accepts and processes external commands over the USB port.
//
// Updating the needles is performed by first taking the time from the
// RTC and breaing it down into a fractional number of hours and a
// fractional number of minutes since the last 12 hour period. It than
// takes the time into a lookup table to get a PWM value to write to 
// the hardware (digitalWrite). Once written, the PWM peripheral in the
// Arduino generates a PWM signal of to 0 5 volts with duty cycles of
// 0 to 100% based on the value written. This PWM value is sent directly
// to the 0 to 5 volt analog meters. Because the meters have a fairly
// long time constant no filtering is needed to average out the PWM
// signal into a smooth voltage.
//
// Lookup tables were calibrated using calibration mode. See section
// on command interface. The PWM values were sent in till values were
// found that matched the lines on the meter dials.
//
// The buttons are monitored by polling them at 10Hz. At this rate, 
// debouncing is not needed and therefore simplyfies the code. A variable
// rate of time slew is provided to allow for fine adjustments but also
// to allow the clock to be set manualy in a finite amount of time.
//
// The command interface receives bytes at a time when they are 
// available on the "serial" port. This serial port is provided as a
// virtual serial port via the USB driver. The correct driver is
// normaly loaded by Windows 10, but if not can be found on the Sparkfun
// website. Each byte is appended the the previous till such time as
// a carriage return is found. The string is then converted to uppercase
// and the first token found. The first token is checked against the
// possible legal commands for processing.
//
// Commands are:
// T hh mm   - Set time hh mm
// S reg val - Set reg val
// R reg     - Read reg
// D         - Dump RTC registers
// C         - Toggle calibration mode
// P # #     - Send raw PWM ralues to meters
// V         - Toggle verbose mode
// ?         - List list
//
//*********************************************************************

//*********************************************************************
// Includes
//*********************************************************************
#include <SPI.h>
#include "SparkFunDS3234RTC.h"

//*********************************************************************
// Defines
//*********************************************************************



// Assign pins
#define RTC_CS_PIN     18     // Real Time Clock chip select
#define HOUR_PWM_PIN   5      // PWM pin to drive hour meter
#define MINUTE_PWM_PIN 6      // PWM pin to drive minute meter
#define FWD_BTN_PIN    3      // Forward button pin
#define REV_BTN_PIN    4      // Reverse button pin

// Convieneance defines
#define SEC_2_MS   (1000L)    // Number of milliseconds in a second
#define MIN_2_SEC  (60L)      // Number of seconds in a minute
#define HOUR_2_MIN (60L)      // Number of minutes in an hour
#define DAY_2_HOUR (24L)      // Number of hours in a day

// Update time definitions
#define SAMPLE_PERIOD (100)   // 10 Hz
#define UPDATE_PERIOD (10 * SEC_2_MS)  // Update needles every 10 seconds



//*********************************************************************
// Global variables
//*********************************************************************
unsigned long updateTime;   // Time to update the needles
unsigned long sampleTime;   // Time to pool the switches
bool          verbose = false;  // Verbose mode
bool          changed = false;  // Time changed and we need to update the needles
bool          calMode = false;  // Calibration mode

// Calibrated by Skye on 2/24/2018
// Lookup table data for hours to PWM value
float         hIv[] = {0, 1,  2,  3,  4,  5,   6,   7,   8,   9,   10,  11,  12};
float         hDv[] = {2, 22, 42, 63, 83, 104, 125, 146, 167, 187, 208, 230, 253};
// Lookup table data for minutes to PWM value
float         mIv[] = {0, 5,  10, 15, 20, 25,  30,  35,  40,  45,  50,  55,  60};
float         mDv[] = {0, 21, 42, 63, 84, 105, 126, 147, 169, 189, 210, 230, 250};



//*********************************************************************
// Prototypes
//*********************************************************************
void          setTargetTimes(void);
void          commandParser(void);
float         lookup(float dvTbl[], float ivTbl[], unsigned int nTbl, float x);

//*********************************************************************
// Define a reset function at address 0.
// Calling this will reboot the arduino
//*********************************************************************
void(* resetFunc) (void) = 0;


//*********************************************************************
// Arduino initialization routine.
// Gets called once on power up by the 'OS'
//*********************************************************************
void setup(void)
{
  
    unsigned long  tmp;

    // Set the target time for first update
    setTargetTimes();

    // Setup the Real Time Clock and provide the CS we are using
    rtc.begin(RTC_CS_PIN);

    // Set pin modes for PWM meter pins
    pinMode(MINUTE_PWM_PIN, OUTPUT);
    pinMode(HOUR_PWM_PIN,   OUTPUT);

    // Set pin modes for GPIO pins
    pinMode(FWD_BTN_PIN, INPUT_PULLUP);
    pinMode(REV_BTN_PIN, INPUT_PULLUP);

    // initialize serial port
    Serial.begin(115200);

    // Wait for serial port to initialize
    delay(1000);

    // Force a meter update right away
    changed = true;

    // Print welcome message
    Serial.println("Starting");

}


//*********************************************************************
// Main Arduino processing loop
// Gets called from a tight loop by the 'OS'
//*********************************************************************
void loop(void)
{
    static int     fwdCnt, revCnt;
    
    unsigned long  osTime;
    unsigned long  h, m, s, t;
    float          hf, mf;
    unsigned long  tmp;
    long           tl;
    int            fwdState, revState;
    unsigned long  hPwm;
    unsigned long  mPwm;


    // Run command parser 
    commandParser();
    
    // Get current OS time in milliseconds since power on
    osTime = millis();

    // OS time rolls over every 40+ days. To keep safe, we will force
    // a reboot when we get close. A bit of a hack, but...
    if (osTime > 0xffffff00) resetFunc();

    // Is it time to update the meters?
    // Either due to a scheduled update or a request from elsewhere
    if ((osTime >= updateTime) || changed)
    {
        // Set the time for the next update
        updateTime += UPDATE_PERIOD;

        // Get the time
        rtc.update();
        mf = (float)rtc.minute() + (float)rtc.second()/60.0L;
        hf = (float)(rtc.hour() % 12) + mf/60.0;   // Reduce to 12 hour range

        // Convert hours to voltage
        hPwm = lookup(hDv, hIv, 13, hf);

        // Covert to voltage via lookup table
        mPwm = lookup(mDv, mIv, 13, mf);

        // If we are not calibrating, update the meters
        if (!calMode)
        {
            // Update meters
            analogWrite(HOUR_PWM_PIN,   hPwm);
            analogWrite(MINUTE_PWM_PIN, mPwm);
        }

        // If we are in verbose mode, or this is a periodic update
        // (Not an update from changing the time)
        if (!changed && verbose)
        {
            Serial.print(" h: ");        Serial.print(hf); 
            Serial.print(" m: ");        Serial.print(mf); 
            Serial.print(" hPwm: ");     Serial.print(hPwm);
            Serial.print(" mPwm: ");     Serial.print(mPwm);
            Serial.println("");
        }

        // Clear changed flag
        changed = false;

    }
    


    // If it is time to sample the GPIO pins
    if (osTime >= sampleTime)
    {
        // Set next time to update
        sampleTime += SAMPLE_PERIOD;

        // Read state of the two switches
        fwdState = digitalRead(FWD_BTN_PIN);
        revState = digitalRead(REV_BTN_PIN);

        // If forward button has been pressed
        if (fwdState == LOW)
        {

            // Get the time
            rtc.update();     // Force an RTC update
            s = rtc.second();
            m = rtc.minute();
            h = rtc.hour();
            t = s + m*MIN_2_SEC + h*HOUR_2_MIN*MIN_2_SEC;
            
            // Set acceleration variables
            fwdCnt++;
            revCnt = 0;
            if (fwdCnt > 400) fwdCnt = 400;
            
            // If just the first 5 seconds the button has been down
            if (fwdCnt < 5 * 10)
            {
                t += 10;
                
            // If the button has been pressed a while
            } else {
                t += fwdCnt - 5*10 + 10;
            }
            
            changed = true;
            Serial.println("Fwd pushed");
            
        // if backward button is pressed
        } else if (revState == LOW)
        {
          
            // Get the time
            rtc.update();
            s = rtc.second();
            m = rtc.minute();
            h = rtc.hour();
            t = s + m*MIN_2_SEC + h*HOUR_2_MIN*MIN_2_SEC;         

            // Set acceleration variables
            fwdCnt = 0;
            revCnt++;
            if (revCnt > 400) revCnt = 400;
          
            // If just the first 5 seconds the button has been down
            if (revCnt < 5 * 10)
            {
                t -= 10;
                
            // If the button has been pressed a while
            } else {
                t -= revCnt - 5*10 + 10;
            }
            
            changed = true;
            Serial.println("Rev pushed");
            
        // Neither button is pressed    
        } else
        {
          revCnt = 0;
          fwdCnt = 0;
        }

        // If the time was changed, set the RTC
        if (changed)
        {
            // Set the RTC with new time
            s = t % MIN_2_SEC;
            m = (t / MIN_2_SEC) % 60L;
            h = (t / MIN_2_SEC / HOUR_2_MIN) % 12L;
            rtc.setSecond(s);
            rtc.setMinute(m);
            rtc.setHour(h);

            // Leave changed flag set so that we update the meters
        }
    }
}



//*********************************************************************
// Set the target times for:
//   meter update
//   RTC read time
//*********************************************************************
void setTargetTimes(void)
{
    unsigned long osTime;

    osTime = millis();

    // Set the target time for next updates. Stagger
    updateTime       = osTime + 0;
    sampleTime       = osTime + 1;
}



//*********************************************************************
// Lookup routine
// @param dvTbl - Array of dependant values
// @param ivTbl - Array of independant values
// @param nTbl  - Number of elements in the arrays
// @param x     - Independant variable to lookup
//*********************************************************************
float lookup(float dvTbl[], float ivTbl[], unsigned int nTbl, float x)
{
    int i;
    float retval = 0.0;
    float slope;

    // Do for each segment in the array
    for (i=0; i<nTbl-1; i++)
    {
      // If this segment brackets our value
      if ((x >= ivTbl[i]) && (x <= ivTbl[i+1]))
      {
        // Compute slope of segment
        slope = (dvTbl[i+1] - dvTbl[i]) / (ivTbl[i+1] - ivTbl[i]);
        // Compute value
        retval = dvTbl[i] + (x - ivTbl[i]) * slope;
        break;
      }
        
    }

    return retval;
}




//*********************************************************************
// Run command parser 
//*********************************************************************
void commandParser(void) 
{
  char        c;        // Current character
  char        *p;       // Pointer to token
  uint8_t     reg;      // Register number
  uint8_t     val;      // Value of register contents
  uint8_t     h, m;     // Hours and minutes
  static char cmd[80];  // Current command
  static int  icmd = 0; // Position in current command


  // If serial data is available
  if (Serial.available() > 0) 
  {
   
    // Read the next byte
    c = Serial.read();
   
    // If the end of a command
    if ((c == '\n') || (c== '\r')) 
    {
      
      // Null terminate the command
      cmd[icmd] = 0;

      // Convert to upper case
      for(int i = 0; cmd[i]; i++){
        cmd[i] = toupper(cmd[i]);
      }       
     
      // Parse out the first token
      p = strtok(cmd, " ");

      // T - Set time HH MM
      if (strcmp(p, "T") == 0) 
      {
        p = strtok(NULL, " ");
        h = atoi(p);
        p = strtok(NULL, " ");
        m = atoi(p);

        Serial.print("H: "); Serial.print(h);
        Serial.print(" M: "); Serial.print(m);
        Serial.println("");
        
        // Set the RTC
        //          S  M  H  WD D  M  Y
        rtc.setTime(0, m, h, 1, 1, 1, 18);

        // Force an update
        changed = true;
         
      // R - Read register Reg
      } else if (strcmp(p, "R") == 0) 
      {
         p = strtok(NULL, " ");
         reg = atoi(p);
         
         // Read register
         val = rtc.spiReadByte(reg);
         Serial.print("reg, ");
         Serial.print(reg);
         Serial.print(" = ");
         Serial.print(val);
         Serial.println("");

      // S - Set register Reg Val
      } else if (strcmp(p, "S") == 0) 
      {
        p = strtok(NULL, " ");
        reg = atoi(p);
        p = strtok(NULL, " ");
        val = atoi(p);
         
        // Set register
        rtc.spiWriteByte(reg, val);
         
      // D - Dump registers
      } else if (strcmp(p, "D") == 0) 
      {
        for (reg=0; reg <= 0x19; reg++)
        {
          val = rtc.spiReadByte(reg);
          Serial.print("reg, ");
          Serial.print(reg);
          Serial.print(" = ");
          Serial.print(val);
          Serial.println("");
        }
        Serial.println("");
         

      // V - Toggle verbose mode
      } else if (strcmp(p, "V") == 0) 
      {
         
        if (verbose) 
        {
          verbose = false;
          Serial.println("Verbose mode off");
        } else {
          verbose = true;
          Serial.println("Verbose mode on");
        }
        
      // C - Toggle calibration mode
      } else if (strcmp(p, "C") == 0) 
      {
         
        if (calMode) 
        {
          calMode = false;
          changed = true;
          Serial.println("Cal mode off");
        } else {
          calMode = true;
          changed = true;
          Serial.println("Cal mode on");
        }
        
      // P - Set register Reg Val       
      } else if (strcmp(p, "P") == 0) 
      {
        unsigned long  h,m;

        p = strtok(NULL, " ");
        h = atoi(p);
        p = strtok(NULL, " ");
        m = atoi(p);
         
        // Update meters
        analogWrite(HOUR_PWM_PIN,   h);
        analogWrite(MINUTE_PWM_PIN, m);
         
      // ? - Help
      } else if (strcmp(p, "?") == 0) 
      {
        Serial.println("Usage:");
        Serial.println(" T hh mm   - Set time hh mm");
        Serial.println(" S reg val - Set reg val");
        Serial.println(" R reg     - Read reg");
        Serial.println(" D         - Dump RTC registers");
        Serial.println(" C         - Toggle calibration mode");
        Serial.println(" P # #     - Send raw PWM ralues to meters");
        Serial.println(" V         - Toggle verbose mode");
         
      // Not a valid command
      } else {
        Serial.println("Bad command");
      }
     
      // Now that we parsed the command, reset for next command
      icmd=0;

    // If this is not a terminator
    } else 
    {
       
      // Append the character to the command buffer
      cmd[icmd] = c;
      icmd++;
      if (icmd >= 80) 
      {
        Serial.println("Command buffer overflow");
        icmd = 0;
      }
    } // end terminator
  } // end character available
}

// end of file
