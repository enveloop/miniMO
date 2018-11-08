/*
//**************************
//*      miniMO ADSR       *
//*   2016-18 by enveloop  *
//**************************
//
   http://www.minimosynth.com
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

I/O
  1&2: Outputs - control voltage (usually for amplitude)
  3: Input - modulates the same parameter as the knob, at any given time
  4: Input (default)- gate (note ON/OFF) / Output (autotrigger) - trigger 

TRIGGER MODES
  SLAVE (default)
  The ADSR outputs the envelope when it detects an external trigger through I/O 4
 
  AUTOTRIGGER 
  The ADSR outputs the envelope continuosly, acting like an LFO with shape control
    -To set the ADSR to Autotrigger Mode, turn the module ON while pressing the button, and release the button after the battery check 

OPERATION
  Knob: modify the selected parameter
    -When you hop between parameters, miniMO waits until you reach the value it has stored to start effecting changes
  Single click: toggle between the available parameters: Attack Length, Decay Length, Sustain Level, Release Length
    -The LED blinks 1 to 4 times depending on the parameter selected (1-A, 2-D, 3-S, 4-R)
  Double click: trigger the ADSR once, manually
  Triple click: bypass the ADSR
    -The LED turns OFF
    -External triggers are ignored
    -I/O 1-2 both output max value
      -If you have the ouput connected to an OSCillator's volume input, you will hear the oscillator like if the ADSR was not connected   
  
BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
*/

#include <avr/io.h>
//#include <avr/eeprom.h>
#include <util/delay.h>

volatile unsigned int globalTicks;

//calibration (use with sequencer)
bool calibrating = false;

//mode
bool autoTrigger = false;

//output
int envelopeValue;

//button interrupt
volatile bool inputButtonValue;

//envelope stage control;
bool readyToAttack = true;
bool readyToRelease = false;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
int additionalClicks = 0;      //variable to see how many times we click after the first

bool parameterChange = false;
byte controlInput;

int currentStep = 0;

const int attackLevel = 255; 

int ADSR[] = {
  127,   //attackLength
  127,   //decayLength  
  127,   //sustainLevel
  127   //releaseLength 
};

//////////USER EDITING ENCOURAGED
const byte lengthFactor = 4;  //length values are eight bits; we right shift them by this value to make them shorter (for instance, with value 4 the legnths' range is 0 to 15) 
/////////////////////////////////
  
void setup() {
  //disable USI to save power as we are not using it
  PRR = 1<<PRUSI;

  //set LED pin and check the battery level
  pinMode(0, OUTPUT); //LED
  checkVoltage();
  
  ADCSRA = (1 << ADEN);             //reset ADC Control (ADC Enable 1, everything else 0)
  ADCSRA |= (1 << ADPS2);           //set adc prescaler  to 16 for 500kHz sampling frequency (8 also works well but is noisier). 500/13 cycles per  sample = 38.4 Khz
  
  ADMUX = 0;                      //reset multiplexer settings
  ADMUX |= (1<<ADLAR);            //left-adjust result (8 bit conversion)
  ADMUX |= (1 << MUX1) | (1 << MUX0); //control input
  ADCSRA |=  (1<<ADSC);           // start next conversion

  pinMode(1, INPUT);  //digital input (push button)
  pinMode(3, INPUT);  //analog- freq input (knob plus external input 1)
  pinMode(2, INPUT);  //analog- pulse (external input 2)
  pinMode(4, OUTPUT); //output
  
  //disable digital input in pins that do analog conversion
  DIDR0 = (1 << ADC2D) | (1 << ADC3D); //PB3,PB4     
  
  //set clock source for PWM -datasheet p94
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1

  cli();                               // Interrupts OFF (disable interrupts globally)
  
  //PWM Generation -timer 1
  GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb4, compare with OCR1B (see interrupt below), reset on match with OCR1C
  OCR1C  = 0xff;                         // 255
  TCCR1  = (1 << CS10);                  // no prescale
  
  //Timer Interrupt Generation -timer 0
  TCCR0A = (1 << WGM01);               // Clear Timer on Compare (CTC) with OCR0A
  TCCR0B = (1 << CS01);                // prescaled by 8
  OCR0A = 0x64;                        // 0x64 = 100 //10000hz - 10000 ticks per second https://www.easycalculation.com/engineering/electrical/avr-timer-calculator.php
  TIMSK = (1 << OCIE0A);               // Enable Interrupt on compare with OCR0A
  
  //Pin Interrupt Generation
  GIMSK |= (1 << PCIE);    // Enable 
  PCMSK |= (1 << PCINT1);  // on pin 1
  
  sei();                               // Interrupts ON (enable interrupts globally)
  
  //Trigger mode: external or auto
  if (digitalRead(1) == HIGH) {        //If we are pressing the button
    digitalWrite(0, HIGH);             //LED ON
    _delay_ms(2000);                                      
    digitalWrite(0, LOW);
    if (inputButtonValue == 0) {
      autoTrigger = 1;                 //If we released the button, set Autotrigger mode
      pinMode(2, OUTPUT);              //Trigger output
    }
  }
  else {
    autoTrigger = 0;                //external trigger mode 
    pinMode(2, INPUT);              //Trigger input
  }
  digitalWrite(0, HIGH); // turn LED ON
}

//PIN Interrupt
ISR(PCINT0_vect) {                 //PIN Interruption - has priority over Timer 0; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;  //Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

//Timer0 interrupt
ISR(TIMER0_COMPA_vect) {              //10000 ticks per second
  setParameter();
  globalTicks++;
}

void loop() {
  checkButton();
  if (!calibrating){
    if (!autoTrigger) triggerADSR(); 
    else retriggerADSR();
  }
}

void setParameter(){
  controlInput = ADCH;
  if (parameterChange) ADSR[currentStep] = controlInput;
  else if ((controlInput & 0xF0) == (ADSR[currentStep] & 0xF0)) {  //checks the control input against stored value. & 0xF0 is a mask to ignore the 4 lower bits and use less resolution (works better)  
    parameterChange = true;                                        //If the value is the same (because we have moved the knob to the last known position for that parameter),it is ok to change the value :)
  }
  ADCSRA |=  (1<<ADSC);
  
}

void manualTrigger() {
   readADS();
   _delay_ms(150);
   readR();
}

void triggerADSR() {                      //triggers the envelope when it receives a pulse in the designated input   
  if (PINB & 0x04) {                    //Reads button (digital input1, the third bit in register PINB. We check the value with & binary 100, so 0x04). digitalRead(2) = HIGH 
    if (readyToAttack){ 
      readyToAttack = false;
      readADS();
      readyToRelease = true;
    }
  }  
  else{
    if (readyToRelease){
      readyToRelease = false;
      readR();
      readyToAttack = true;
    }
  }  
}

void retriggerADSR() {                      //triggers the envelope continuously
  if (readyToAttack){ 
    readyToAttack = false;
    digitalWrite(2, HIGH);                  //sends a trigger signal through I/O4
    readADS();
    digitalWrite(2, LOW);
    readyToRelease = true;
  }
  else if (readyToRelease){
    readyToRelease = false;
    readR();
    readyToAttack = true;
  }
} 

void readADS() {
  int attackLength = ADSR[0] >> lengthFactor; 
  int decayLength = ADSR[1] >> lengthFactor;
  int sustainLevel = ADSR[2]; 
  
  //ATTACK
  if (attackLength == 0) OCR1B = attackLevel;
  else {
    globalTicks = 0;
    for (envelopeValue = 0; envelopeValue <= 255; ){    
      OCR1B = envelopeValue;  
      if (globalTicks == attackLength) {
        envelopeValue++;
        globalTicks = 0;
      } 
    }
  }
  //DECAY                                                
  globalTicks = 0;
  if ((decayLength == 0) || (sustainLevel == attackLevel)) OCR1B = sustainLevel;
  else{
    if (sustainLevel < attackLevel){
      for (envelopeValue = attackLevel; envelopeValue >= sustainLevel; ){   
        OCR1B = envelopeValue;
        if (globalTicks == decayLength) {
        envelopeValue--;
        globalTicks = 0;
        } 
      }
    }
  }
  //SUSTAIN --nothing to do, we keep the last value until the "note off" trigger
}

void readR() {
  int sustainLevel = ADSR[2];
  int releaseLength = ADSR [3] >> lengthFactor;
  
  //RELEASE
  if (releaseLength == 0) OCR1B = 0;
  else {
    OCR1B = sustainLevel;
    globalTicks = 0;
    for (envelopeValue = sustainLevel; envelopeValue >= 0;){
      if (!autoTrigger) {                                         //NORMAL ADSR ONLY
        if (PINB & 1 << 2){                                       //if during R stage there's a new trigger, silence and exit
         OCR1B = 0;
         return;
        }
      }
     OCR1B = envelopeValue;
       if (globalTicks == releaseLength) {
         envelopeValue--;
         globalTicks = 0;
       }   
    }
  }
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    button_delay++;
    _delay_ms(10);
  }
  if (inputButtonValue == LOW) {
    beenDoubleClicked = false;
    if (button_delay > 0) {
      bool hold = true;
      while (hold) {
        bool previousButtonState = inputButtonValue; //see if the button is pressed or not
        
        _delay_ms(1);
       
        button_delay_b++;                                                  //fast counter to check if there are more presses
        if ((inputButtonValue == HIGH)&& (previousButtonState == 0)) {   
          additionalClicks++;                                              //if we press the button and we were not pressing it before, that counts as a click
        }
        
        if (button_delay_b == 300) {
          if (additionalClicks == 0){    //button released (regular single click)         
            
            parameterChange = false;                       
          
            currentStep++;                          //cycles through the steps
            if (currentStep >= 4) currentStep = 0;   
            flashLEDSlow(currentStep + 1);              
            
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            hold = false; 
          }
          else if (additionalClicks == 1) {             //button pressed again (double click),         
            
            manualTrigger();
           
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
          }
          else if (additionalClicks > 1 ) {                 //button pressed at least twice (triple click or more)
            
            if (calibrating == true){
              digitalWrite (0, HIGH);
              OCR1B = 0; 
              calibrating = false;
            }
            else {
              digitalWrite (0, LOW);
              OCR1B = 255;
              calibrating = true;
            }

            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
          }
        }
      }
    }
  }
}

void checkVoltage() {                   //voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5
                                        //we measure a fixed value of 1.1 against Vcc, so the lower the measurement, the higher Vcc
  ADMUX = (0 << REFS1) | (0 << REFS0);  //Vcc as reference
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

void flashLEDSlow(int times) {
  for (int i = 0; i < times; i++){
    _delay_ms(70);
    digitalWrite(0, LOW);
    _delay_ms(70);
    digitalWrite(0, HIGH);
  }
}

void flashLED (int times, int gap) {     //for voltage check only (uses regular delay)
  for (int i = 0; i < times; i++)
  {
    digitalWrite(0, HIGH);
    delay(gap);
    digitalWrite(0, LOW);
    delay(gap);
  }
}
