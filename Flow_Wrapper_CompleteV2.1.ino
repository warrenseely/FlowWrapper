  /*V2.1 gives priming of the feeder chain on startup with mints prior to running
   * Also introduces LCD display communication on the tx channel of USB
   * Works simultaneously with the USB plugged in, although LCD commands show as garbage
   * Note that overloaded function writeLCD() can only print those types of data it is overloaded for
   * Also introduces a 1.5 minute startup delay on power up to allow the VFDs to unscramble their brains
   * Also introduces checks to ensure jaws make complete cycle during startup to avoid weird pwm bugs resulting from less than full cycle
   * Also disables pin change interrupts when stop button pressed, reenables when start button pressed UNTESTED
   * 
   * V2.0 introduced PID control of the feeder chain to match the paper speed. 
  Also introduced Timer1 set as 16bit to enable 10bit pwm on pins 9 and 10(jaw and feeder pwm) for greater resolution(1024 steps vs 255)
  Also utilizes encoders on paper drive and feeder drive to match speeds correctly utilizing the PID control.
  This version introduces a timed-loop control for consistent response time for both button commands and run operation.
  includes live printout of speed errors, and current PID scalars along with ability to adjust during run operation WITHOUT SAVING
  Also splits the jaw and paper register interrupts into separate pin change ISRs.
  Note that feeder speed is adjusted by PID, but jaw speed is hard coded in this version. 
  Jaw speed is still stop/go control.
  Note that the PID runs off of incrementing pulses on the encoders; 
  Encoder counts utilize LONG int, which should give 1,491.3 hours of continouos run time(without pressing stop) prior to overrun*/

//Encoder wiring:
//Green: A-phase
//White: B-phase
//Red: +5-24v
//Black: Ground

/*Paper speed is MASTER
  Jaws run based on paper speed
  feeder chain runs based off paper speed*/


/*******************IMPORTANT!!!!!********************
*************Pkg dimensions are in METRIC*************
*******************************************************/
// -- > https://github.com/JChristensen/Button/blob/master/Button.cpp
#include <Button.h> //include this library for convenient reading of the buttons
#include <Wire.h> //serial communication

#define SettingsBtn 6 //settings button
#define StopBtn A1 //stop button
#define StartBtn A2

#define JawEnable_PIN A3 //on/off for jaw vfd control
#define PaperEnable_PIN A4 //on/off for paper vfd control
#define FeederEnable_PIN A5 //on/off for feeder chain vfd control

#define RegisterEncoder_PIN 2 //D2, register encoder sensor
#define FeederEncoder_PIN 3 //D3, feeder chain encoder sensor
#define JawPos_PIN 4 //input for the jaw position; HIGH = HOME, LOW = RETURN TO HOME
#define RegisterSensor_PIN 12 //triggers jaw to cut
#define FeederFinger_PIN A0 //A0 is a sensor to detect feeder chain fingers for homing and adjusting speed during run
#define PattySensor_PIN 7 //D7 is a sensor to detect patties upon startup only

#define jawPWM 9 //D9 speed output signal for jaws; VFD terminal 12; set for 10 bit control 4/7/18
#define feederPWM 10 //pwm output to control speed of feeder drive; set for 10 bit control 4/7/18
#define paperPWM 11 //pwm output to control speed of paper drive
//D5,8,13 unused

//constant motor defines
const int stopAll = 1;

//button defines
#define PULLUP true
#define INVERT true
#define DEBOUNCE_MS 20

//ISRs
volatile int paperHome = 0, paperhomeflag = 0, fingerHomeFlag = 0, pkgcount = -9, lastpkgcount = 0;
volatile int jawhomeflag = 0;
volatile long int PaperCount = 0, FeederCount = 0; //counters for the encoders
volatile long int pkgTime = 0, lastPkgTime = 0;

////length of register mark in mm, package length in mm, packages per minute(default to enable homing at startup
////packages per minute = jaw RPMs
//const float pkgLength = 111;
//const float maxPkgPerMinute = 580; //this is used for converting the pkgPerMinute to voltage output to control jaws
//                             //motor rpm @ 60Hz = 1750 plus roughly a 3:1 reduction on jaw drive gives ~580 rpm per minute max.
//                             //NOTE: Can increase max motor rpm by increasing vfd max frequency above 60Hz

float pkgPerMinute = 0;
long feeder_count = 0, paper_count = 0, feeder_error = 0;
int jaw_speed = 0, feeder_speed = 0, paper_speed = 10;
int setpoint = 5, setBtnFlag = 0, startupFlag = 0, StartBtnState = 0, StopBtnState = 1, currently_running = 0;
int homeflag = 0; //for startup

//timed loop variables
long int oldTime = 0;
int loop_time = 100, startupDelay = 2000;
unsigned long startTime = 0;

//for PID loop
long int integral = 0; 
long lastfeeder_error = 0;
float kp = 0.08, ki = 0.006, kd = 0.6; //kp = 0.08, ki = 0.006, kd = 0.6 good start

//scalars for startup at close to correct speeds. based on paper_speed = 10. feeder verified 4/4/18
//bitScale converts from the 8-bit pwm write for paper speed to the 10-bit pwm write for jaw and feeder speed
volatile float timingScalar = 0;
const float jawScalar = 3.6, feederScalar = 5.38, bitScale = 4.01, countScalar = 1.343; 
long int fingerTimecopy = 0, lastPkgTimecopy = 0;    
float timingScalarcopy = 0; 
const int offset = 300; //timing offset between finger and register
volatile long int timeDifference = 0, fingerTime = 0; //time difference between register and finger
volatile int ptf = 0, tdc = 0;
int resetFlag = 0;

//declare buttons
 Button settingsBtn(SettingsBtn, PULLUP, INVERT, DEBOUNCE_MS);    //Declare settings button on A0; 
 Button stopBtn(StopBtn, PULLUP, INVERT, DEBOUNCE_MS);    //Declare stop button on A1; 
 Button startBtn(StartBtn, PULLUP, INVERT, DEBOUNCE_MS);    //Declare start button on A2; 
 
void setup() 
{
  //pin change interrupt(for A0, feeder finger detection)
  PCMSK1 |= bit (PCINT8); //want pin A0
  PCIFR |= bit (PCIF1); //clear any outstanding interrupts
  //PCICR |= bit (PCIE1); //enable pin change interrupts for port C; enabled when start button pressed
  
  //pin change interrupt (for D12, paper register detection)
  PCMSK0 |= bit (PCINT4); //want pin D12
  PCIFR |= bit (PCIF0); //clear any outstanding interrupts
  //PCICR |= bit (PCIE0); //enable pin change interrupts for port B; enabled when start button pressed
  
  // pin change interrupt (for D4, jaw control)
  PCMSK2 |= bit (PCINT20);  // want pin D4
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  //PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7; enabled when start button pressed

  pinMode(RegisterEncoder_PIN, INPUT_PULLUP); //interrupt counters for the encoder pulses; set to input pullup to enable pullup resistors
  pinMode(FeederEncoder_PIN, INPUT_PULLUP); //interrupt counters for the encoder pulses; set to input pullup to enable pullup resistors
  pinMode(JawPos_PIN, INPUT_PULLUP);
  pinMode(RegisterSensor_PIN, INPUT_PULLUP); 
  pinMode(PattySensor_PIN, INPUT_PULLUP); //senses a patty upon startup for homing
  pinMode(FeederFinger_PIN, INPUT); //senses the feeder finger on startup for homing
  pinMode(jawPWM, OUTPUT); //jaw speed
  pinMode(paperPWM, OUTPUT); //paper speed
  pinMode(feederPWM, OUTPUT); //feeder chain speed
  pinMode(JawEnable_PIN, OUTPUT); //jaw on/off
  pinMode(PaperEnable_PIN, OUTPUT); //paper on/off
  pinMode(FeederEnable_PIN, OUTPUT); //feeder chain on/off
  
  attachInterrupt(digitalPinToInterrupt(RegisterEncoder_PIN), paperEncoderISR, CHANGE); //triggers on change, counts paper encoder pulses
  attachInterrupt(digitalPinToInterrupt(FeederEncoder_PIN), feederEncoderISR, CHANGE); //triggers on change, counts feeder chain encoder pulses

  //PWM rate settings Adjust to desired PWM Rate
  ////TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)

// Set pwm clock divider
TCCR1B = 0x09;  // set Control Register to no prescaling 
                  // WGM02 = 1
TCCR1A = 0x03;  // set WGM01 and WGM00 to 1 (10 bit resolution--> 0 to 1024)
  
  //set up communication  
  Wire.begin();
  Serial.begin(9600);
  writeLCD("WAIT");
  //Serial.println("Hal9000 V2.1 Online");
  
  //put this turn-off here after the fireup print statement, works best for ensuring no random movement
  //shut all vfds off now to prevent random movement during startup
  PORTC &= ~(1 << PORTC3); //shut off vfd for jaws(faster than digitalWrite()
  PORTC &= ~(1 << PORTC4); //shut off vfd for paper
  PORTC &= ~(1 << PORTC5); //shut off vfd for feeder chain

  unsigned long Delay = millis(); //preserve current time
  //while(millis() - Delay < 90000); //90,000 ms = 1.5 minute delay to allow VFDs to reset after power on
  writeLCD("Hal9000 V2.1 Online");
}

void loop() 
{
  //read the buttons every time we loop, outside the timed loop to give best chance of catching them
  stopBtn.read(); //read stop button
  startBtn.read(); //read start button
  settingsBtn.read(); //read the status of the settings button
  
  button_results(); //deal with which buttons were read
  
  unsigned long currentTime = millis();
  if((unsigned long)(currentTime - oldTime) >= loop_time) //we are on a timed loop every 100 ms
  {
    oldTime = millis(); //get the current time to reset timed loop
    
    if(StartBtnState && !StopBtnState && !startupFlag) //we have a run condition and are not starting up 
    {         
        PCICR &= ~(1 << PCIE1);//disable finger interrupt
        timingScalarcopy = timingScalar; //copy to a non volatile for use
        tdc = timeDifference;
        //lastPkgTimecopy = pkgTime;
        //fingerTimecopy = fingerTime;
        PCICR |= bit (PCIE1);//enable finger interrupt
        
        PCICR &= ~(1 << PCIE0); //disable pin change interrupts for port B
        PCICR &= ~(1 << PCIE2);    // disable pin change interrupts for D0 to D7 //halt the encoder interrupts while making a copy
        feeder_count = FeederCount; //copy the pulse count into a non volatile variable to prevent modification during use; these are long ints, they are reset on startup
        paper_count = PaperCount; //copy the pulse count into a non volatile variable to prevent modification during use; these are long ints, they are reset on startup
//        if(resetFlag) //reset encoder counters every so often to keep them from becoming mega large as fast
//        {
//          resetFlag = 0; //reset flag
//          FeederCount -= 10000L; //knock the counts down to keep them from growing super large as fast; check below to ensure this does not produce negative numbers
//          PaperCount -= 10000L;
//        }
        PCICR |= bit (PCIE0); //enable pin change interrupts for port B
        PCICR |= bit (PCIE2);    // enable pin change interrupts for D0 to D7 //re-enable the encoder interrupts after copying
        //float temp = (paper_count + (80 + timingScalarcopy)); //there is a difference of 80 pulses per package between the feeder and paper encoders; increase timingScalarcopy to increase feeder speed
        long temp = (paper_count * (countScalar + timingScalarcopy)) + 0.5; //increase timingScalarcopy to increase feeder speed
        feeder_error = temp - feeder_count; //get the difference in the pulse numbers
       
        feederPID(); //calculate the PID to adjust feeder chain speed
        analogWrite(feederPWM, feeder_speed); //adjust the feeder chain speed
      
        pkgPerMinute = 1.0 / (pkgTime * 0.000001667); //packages per minute = 1 package / (# of milliseconds passed * portion of minute passed)

//        if((paper_count > 10010) && (feeder_count > 10010)) //check to make sure there is no negative number
//          resetFlag = 1; //set flag to reduce encoder counters to keep them from becoming mega large as fast
//          
      if((pkgcount > 0) && (pkgcount != lastpkgcount)) //only print pkgcount when it changes, and skip the first 10 since they are empties from machine start
      {
        lastpkgcount = pkgcount; //update
        //writeLCD(pkgCount);
        //writeLCD(timeDifference);
        printData(4); //send information out
        Serial.print(paper_count);
        Serial.print(", ");
        Serial.println(feeder_count);
        //printData(5);
      }
      if(ptf) //for testing. prints every time a finger is seen
      {
        ptf = 0;
        Serial.print("paper PWM: ");
        Serial.print(paper_speed);
        Serial.print("\t");
        Serial.print("timingScalarcopy: ");
        Serial.print(timingScalarcopy*100);
        Serial.print("\t");
        Serial.println(tdc);
      }
    }//end runtime if statement
    else if(StartBtnState && !StopBtnState && startupFlag) //we have a run condition and we are starting up 
    { 
      startupProcedure(); //homing and alignment
    }//end startup if statement
    else if(setBtnFlag && !StartBtnState && StopBtnState && !startupFlag) //settings button pressed and we are not currently running and we are not starting up
    {
      getSettings(); //get new settings
    }//end settings if statement
  }//end timed loop
  
//  livePIDTune(); //for tuning PID and feeder drive speed(DOES NOT SAVE!)
}//end main loop 

/****************************************************************************/
//ISR
ISR (PCINT1_vect) //feeder chain timing 
{
  if(!(PINC & bit (0))) //finger is seen when pin goes low
  {
    if(!fingerHomeFlag) //normal run operation
    {
      timeDifference = (long)(millis() - (pkgTime + offset)); //time difference between paper register and feeder finger, paper register is always ahead slightly so shift with offset
        //fingerTime = millis();
        ptf = 1; //for testing
        //******maybe instead just add or subtract from paper encoder count to adjust speed
      if(timeDifference > 40) //if the time between the register and finger is out of desired range
        timingScalar += 0.001; //speeds up the feeder chain
        //timingScalar += 1; //speed up feeder chain
      else if(timeDifference < -40) //have more room ahead
        timingScalar -= 0.001; //slows down the feeder chain
        //timingScalar -= 1;//slow feeder chain down
    }
    else if(startupFlag == 4)//we are homing and have seen a patty
    {
      fingerHomeFlag = 0; //reset to zero
      PORTC &= ~(1 << PORTC5); //stop feeder chain vfd
    }  
  }
}

//ISR
ISR (PCINT0_vect) //paper register ISR
{
  //hande pin change interrupts for port B here
  //currently set up for D12 ONLY
  //check for the paper register mark
   if(!(PINB & bit (4))) //pin goes LOW when register mark seen
   {
      if(!paperHome) //we are not homing
      {
        analogWrite(jawPWM, jaw_speed); //write the speed of the film during cut
        PORTC |= (1 << PORTC3); //enable the jaws
       
        pkgcount++; //increment the package count
        pkgTime = millis(); //get length of time to get this package
        lastPkgTime = millis(); //update for next trigger 
      }
      else //for paper homing upon startup only
      {
        PORTC &= ~(1 << PORTC4); //we are at home position on startup so stop paper
        paperhomeflag = 1;
        paperHome = 0; //reset to 0; set to 1 in startup routine
      }
   }//end pin low check
}//end paper register ISR

//ISR
ISR (PCINT2_vect) //jaw control ISR
 {
   // handle pin change interrupt for D0 to D7 here
   //currently set up for D4 ONLY
   
   //check for jaw trigger
   if ((PIND & bit (4)))  //if it was change to high, we know jaws are at HOME
   {    
      if(jawhomeflag != -1) //used for startup only
      {
          PORTC &= ~(1 << PORTC3); //we are at home position so shut off jaws and wait for register trigger
          jawhomeflag = 1; //used for startup
      }
       
   }
   else //it was jaw change to low, so we know jaw just finished CUTTING
   {
        int temp = 5.6 * jaw_speed;
        analogWrite(jawPWM, temp); //write 2x the speed during homing 
        if(jawhomeflag == -1) //used for startup only
          jawhomeflag = 2;
   }
 }  // end of PCINT2_vect


void paperEncoderISR() //used to detect beginning of and end of register mark since used for film speed; Pin1 triggers on HIGH
{
  PaperCount++; //increment counter
}//end paperencoder ISR

//ISR
void feederEncoderISR() //used to detect feeder chain fingers Pin2 triggers on HIGH
{
  FeederCount++; //increment counter
}//end feederencoder ISR
