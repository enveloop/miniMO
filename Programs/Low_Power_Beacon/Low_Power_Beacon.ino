/*
//*********************************
//*    miniMO LOW POWER BEACON    *
//*       2017 by enveloop        *
//*********************************
  Inspired by The Annoy-O-Bug by Alex Wulff: https://www.hackster.io/AlexWulff/the-annoy-o-bug-a-chirping-light-up-throwie-37e58a
  Watchdog setup described by Martin Nawrath: http://interface.khm.de/index.php/lab/interfaces-advanced/sleep_watchdog_battery/
  Square wave generation described here: https://playground.arduino.cc/Main/PbSynthCode
  Also uses the Xorshift pseudorandom number generator, described here: http://www.arklyffe.com/main/2010/08/29/xorshift-pseudorandom-number-generator/

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license:
   http://creativecommons.org/licenses/by/4.0/
//

CONCEPT

This program puts miniMO in a low power sleep condition for an adjustable interval of time. 
When miniMO wakes up, it plays one out of several preprogrammed sequences, then goes to sleep again. 

I/O
  1&2: Outputs - signal
  3: Input - frequency or note length modulation / set sleep interval (when pressing button)
  4: Input - sequence change

OPERATION

  Knob: 
    -If the button is not pressed: change frequency (sequences 00 to 02), initial note frequency (sequence 03) or note length (sequence 04)
      -All the tones are square waves
    -If the button is pressed: change sleep interval
    -The LED toggles ON/OFF with every note played
  Button Press: set the sleep interval 
    -To register the change, you must have the button pressed by the end of the current sequence
      -miniMO checks for a button press right before going to sleep
    -Depending on the knob's position, the interval can take values from 64ms to 8 s   
  Finger Tap on both terminals of I/O 4: cycle through the available sequences
    -To register the change, you must have the terminals pressed by the end of the current sequence
    -Available Sequences (in order): 
      -sequence 00: single beep 
      -sequence 01: double beep
      -sequence 03: SOS in Morse code
      -sequence 04: Portamento SFX
      -sequence 05: single random tone 

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

NOTES:

  ON CHANGING THE SEQUENCE
    This program uses I/O 4 as a makeshift extra button.
    The best way to try this feature is to turn miniMO on, wait for a couple beeps, then gently place a finger over BOTH pins of I/O 4, until the sequence changes.
    This is an experimental feature! if for whatever reason it doesn't work for you, please let me know
    To disable this feature,
      comment the line that has the following code: if (analogRead(1) < 700) advanceSeq();
      modify playSeq(value from 0 to 4) to set the initial sequence.
*/

#include <avr/sleep.h>

static byte watchdogInterval = 6;   //intial watchdog sleep interval (1 sec)

int currentSeq = 0;
int lastSeqIndex = 4;       //index of the last sequence available

//random number generator
static unsigned long y32 = 1; //pattern length: 32 bit

void setup() { 

 pinMode(0, OUTPUT); //LED
 pinMode(4, OUTPUT); //outs 1 and 2
 pinMode(3, INPUT);  //analog- freq input (knob plus external input 1)
 pinMode(1, INPUT);  //digital input (push button)

 checkVoltage();
 ADMUX = 0;                      //reset multiplexer settings
 
 setup_watchdog(watchdogInterval);       //0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms, 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
} 

ISR(WDT_vect) {                  //the watchdog ISR must be present even if it's empty. Otherwise, the system resets every time it wakes up 
}

void loop() { 
    
    playSeq(currentSeq); 
    checkButtons();              
    system_sleep();

} 

void checkButtons() {
  if (analogRead(1) < 700) advanceSeq();            //detect finger tap in I/O 4 -experimental! if doesn't work for you, try different values or comment the line
  if(digitalRead(1) == 1){
      watchdogInterval = (analogRead(3) >> 7) + 2;  //values between 2 and 9 (64ms to 8s)
      setup_watchdog(watchdogInterval);
  }
}

void playSeq(int seq) {      
  switch (seq) {
    case 0:
      sequence00();
      break;
    case 1:
      sequence01();
      break;
    case 2:
      sequence02();
      break;
    case 3:
      sequence03();
      break;
    case 4:
      sequence04();
      break;
  }
}

void advanceSeq()
{
 currentSeq++;                                           
 if (currentSeq >= lastSeqIndex + 1) currentSeq = 0;   
}

//***********************SEQUENCES************************

void sequence00() {                      //single beep, frequency set by knob's position, length 30ms
  freqout((analogRead(3) << 2), 30);       
}

void sequence01() {                      //double beep
  freqout((analogRead(3) << 2), 30);
  delay(30);
  freqout((analogRead(3) << 2), 30); 
}

void sequence02() {                      //SOS in Morse Code
  dot(); dot(); dot(); 
  delay(100);
  dash(); dash(); dash();
  delay(100); 
  dot(); dot(); dot();
}

void sequence03() {                     //descending portamento, initial frequency set by knob's position
  int readOut = analogRead(3);
  if (readOut < 70) readOut = 70;       //lower readouts crash the program 
  int initial = readOut << 5;           //readout * 2^5 (just makes the initial frequency fairly high)
  
  for (int i = 0; i < 10; i++) { 
    freqout((initial >> i), 30);        //initial / 2^i
    delay(initial / (initial >> i));
  }
}

void sequence04() {
  freqout(xorshift32(), (analogRead(3) >> 3));  //random frequencies, note legnth set by knob's position
}
//*******************************************************

byte xorshift32(void) {
    y32 ^= (y32 << 7);
    y32 ^= (y32 >> 5);
    return y32 ^= (y32 << 3);  //pattern values: 8 bit
}

//Morse******************************
void dot() {
  freqout((analogRead(3) << 2), 50);
  delay(50);
}

void dash() {
  freqout((analogRead(3) << 2), 150);
  delay(50);
}
//************************************

void freqout(int freq, int t) { 
  int hperiod;    
  long cycles, i; 

  hperiod = 500000 / (freq - 7);          // subtract 7 us to make up for digitalWrite overhead - determined empirically                
  cycles = ((long)freq * (long)t) / 1000;   
  
  digitalWrite(0, HIGH);                  //turn LED ON
  for (i=0; i<= cycles; i++)              //square wave generation
  {  
    digitalWrite(4, HIGH);  
    delayMicroseconds(hperiod); 
    digitalWrite(4, LOW);  
    delayMicroseconds(hperiod - 1);
  } 
  digitalWrite(0, LOW);                   //turn LED OFF                
}

void system_sleep() {
                  
  ADCSRA &= ~(1 << ADEN);              // switch ADC OFF
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  
  sleep_enable();                      // nap time
  sleep_mode();                        

  sleep_disable();                     // wake up when watchdog times out

  ADCSRA |= (1 << ADEN);               // switch ADC ON
}

void setup_watchdog(int ii) {
  
  byte bb;
  bb = ii & 7;                       //for inputs from 0 to 7, nothing changes. If the inputs are 8 or 9, this sets the fourth bit in bb to 0  
  if (ii > 7) bb |= (1 << 5);        //if the inputs are 8 or 9, sets bit 6 in bb to 1
                                     //why is this? see below

  MCUSR &= ~(1 << WDRF);              //clear watchdog reset flag
  WDTCR |= (1 << WDCE) | (1 << WDE);  //timed sequence to change the wathdog configuration (datasheet p.43)
  
  WDTCR = bb;                         //set watchdog prescaler (also clears WDCE: note the equals sign)
                                      //the bits that set the prescaler are not contiguous in the register: from right to left, they take positions 1,2,3, and 6
                                      //the code above took care of that, because if the input is 7 or 8, it sets bit 6 to 1, instead of setting bit 4 to 1
  
  WDTCR |= (1 << WDIE);               // trigger interrupt on time out
}

void checkVoltage() {                   //voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5
                                        //we measure a fixed value of 1.1 against Vcc, so the lower the measurement, the higher Vcc
  ADMUX = (0 << REFS1)|(0 << REFS0);    //Vcc as reference
  ADMUX |= (1 << ADLAR);                //Left adjust result (8 bit conversion stored in ADCH)
  ADMUX |= (1 << MUX3) | (1 << MUX2);   //1.1v input
  delay(250);                           // Wait for Vref to settle
  ADCSRA |= (1 << ADSC);                // Start conversion
  while (bit_is_set(ADCSRA, ADSC));     // wait while measuring
  if (ADCH > 103)                       //approx 2.6
    flashLED(8, 100);
  else
    flashLED(1, 250);
}

void flashLED (int times, int gap) {     
  for (int i = 0; i < times; i++)
  {
    digitalWrite(0, HIGH);
    delay(gap);
    digitalWrite(0, LOW);
    delay(gap);
  }
}
