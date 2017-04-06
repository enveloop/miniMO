/*
//************************
//*      miniMO ADSR     *
//*   2016 by enveloop   *
//************************
//
   http://www.minimosynth.com
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

I/O
  1&2: Outputs - control voltage (usually for amplitude)
  3: Input - modulates the same parameter as the knob, at any given time
  4: Input - gate (note ON/OFF)

OPERATION
  Knob: modify the selected parameter
    -miniMO waits until you reach the last value, then starts applying the new ones
  Single click: toggle between the available parameters: Attack Length, Decay Length, Sustain Level, Release Length
    -the LED blinks 1 to 4 times depending on the parameter selected (1-A, 2-D, 3-S, 4-R)
  Double click: trigger the ADSR once, manually 
  
BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
*/

#include <avr/io.h>
//#include <avr/eeprom.h>
#include <util/delay.h>

volatile unsigned int globalTicks;
volatile int watchdogClicks = 0;

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

bool lengthChange = false;
bool levelChange = false;

int currentStep = 0;

const int attackLevel = 255; 

int ADSR[] = {
  0,   //attackLength
  1,   //decayLength  
  0,   //sustainLevel
  0   //releaseLength 
};
  
bool calibrating = false;

void setup() {
  //disable USI to save power as we are not using it
  PRR = 1<<PRUSI;

  //set LED pin and check the battery level
  pinMode(0, OUTPUT); //LED
  checkVoltage();
  ADMUX = 0;                      //reset multiplexer settings

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
  GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb1, compare with OCR1B, reset on match with OCR1C
  OCR1C  = 0xff;                         //255
  TCCR1  = (1 << CS10);                  // no prescale
  
  GIMSK = (1 << PCIE);    // Enable Pin Change Interrupt
  PCMSK = (1 << PCINT1);  // on pin 1
  
  //Timer Interrupt Generation -timer 0
  TCCR0A = (1 << WGM01);               //Clear Timer on Compare (CTC) with OCR0A
  TCCR0B = (1 << CS01);                // prescaled by 8
  OCR0A = 0x64;                        //0x64 = 100 //10000hz - 10000 ticks per second https://www.easycalculation.com/engineering/electrical/avr-timer-calculator.php
  TIMSK = (1 << OCIE0A);               // Enable Interrupt on compare with OCR0A
  
  sei();                               // Interrupts ON (enable interrupts globally)
  
  digitalWrite(0, HIGH); // turn LED ON
}

//PIN Interrupt
ISR(PCINT0_vect) {                 //PIN Interruption - has priority over Timer 0; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;  //Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

//Timer0 interrupt
ISR(TIMER0_COMPA_vect) {              //10000 ticks per second
  globalTicks++;
}

void loop() {
  if (calibrating == false){
    checkButton();
    setParameter();
    triggerADSR(); 
  }
  else if (calibrating == true){
    
    checkButton();
  } 
}

void setParameter(){
  if (currentStep == 2) setLevel(3); //step 2 -sustain
  else setLength(3); 
}

void setLength(int pin) {
  levelChange = false;
  int lengthRead = analogRead(pin) >> 6 ;   //values between 0-15 
  if(lengthChange == false){
    if(lengthRead == ADSR[currentStep]){
      lengthChange = true;
    }
  }
  else if(lengthChange == true){
    ADSR[currentStep] = lengthRead;
  }
}

void setLevel(int pin) {
  lengthChange = false;
  int levelRead = analogRead(pin) >> 2 ;   //values between 0-255 
  if(levelChange == false){
    if(levelRead == ADSR[currentStep]){
      levelChange = true;
    }
  }
  else if(levelChange == true){
    ADSR[currentStep] = levelRead;
  }
}

void manualTrigger() {
   readADS();
   _delay_ms(150);
   readR();
}

void triggerADSR() {                      //triggers the reading of the envelope when it receives a pulse in the designated input   
  
  if (digitalRead(2) == HIGH) { 
    readyToRelease = true;  
    if (readyToAttack){
      readADS();
      readyToAttack = false;
    }
  }  
  if (digitalRead(2) == LOW) {
    readyToAttack = true;
    if (readyToRelease){
      readR();
      readyToRelease = false;
    }
  }  
}

void readADS() {
  int attackLength = ADSR[0]; 
  int decayLength = ADSR[1];
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
  if (decayLength == 0) OCR1B = sustainLevel;
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
    else if (sustainLevel == attackLevel){
        OCR1B = envelopeValue;
    }
  }
  //SUSTAIN --nothing to do, we keep the last value until the "note off" trigger
}

void readR() {
  int sustainLevel = ADSR[2];
  int releaseLength = ADSR [3];
  
  //RELEASE
  if (releaseLength == 0) OCR1B = 0;
  else {
    OCR1B = sustainLevel;
    globalTicks = 0;
    
    for (envelopeValue = sustainLevel; envelopeValue >= 0;){
     OCR1B = envelopeValue;
       if (globalTicks == releaseLength) {
         envelopeValue--;
         globalTicks = 0;
       }   
    }
  }
  OCR1B = 0;
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    button_delay++;
    _delay_ms(10);
    if (button_delay > 50 &! beenDoubleClicked) {
      beenLongPressed = true;                      //press and hold 

    }
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
        
        if (button_delay_b == 500) {
          if (additionalClicks == 0){           
            if (beenLongPressed) {                    //button released after being pressed for a while (most likely because we were changing the volume)
              beenLongPressed = false;
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
            else {                                    //button released (regular single click)
              
              lengthChange = false;
              levelChange = false;
              
              currentStep++;                          //cycles through the steps
              if (currentStep >= 4) currentStep = 0;   
              flashLEDSlow(currentStep + 1);              
              
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
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

void flashLEDFast(int times) {
  for (int i = 0; i < times; i++){
    _delay_ms(100);
    digitalWrite(0, LOW);
    _delay_ms(100);
    digitalWrite(0, HIGH);
  }
}

void flashLEDSlow(int times) {
  for (int i = 0; i < times; i++){
    _delay_ms(150);
    digitalWrite(0, LOW);
    _delay_ms(150);
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
