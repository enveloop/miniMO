/*
//*************************************
//*    miniMO Noise Random Generator  *
//*         2018 by enveloop          *
//*************************************

  Uses the xorshift pseudorandom number generator,
        as described Here:
http://www.arklyffe.com/main/2010/08/29/xorshift-pseudorandom-number-generator/

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license:
   http://creativecommons.org/licenses/by/4.0/
//

I/O
  1&2: Outputs - noise/grains
  3: Input - frequency/grain density modulation
  4: unused
  
OPERATION
  
  Knob: change frequency (default) or grain density
    -miniMO waits until you reach the value it has currently stored 
  Click: toggle between frequency and density control 
    -The LED blinks once -  frequency control
    -The LED blinks twice - density control     

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

NOTES&TROUBLESHOOTING
  Extra Mode - Randomize All
    -If you select the randomizeMore() function in the main loop, miniMO will randomize all parameters
    -In this mode, there's no live user input
    -Note that there is also a delay() under OCR1C to play with :D
*/


#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

//button interrupt
volatile bool inputButtonValue;
bool controlFrequency = 1;

//random number generator
static unsigned long y32 = 1;   // pattern length: 32 bit 

//freq control reference
bool coarseFreqChange = false;  
byte potPosFreqRef = 255;       // max

//grain control reference
bool coarseGrainChange = false;
byte potPosGrainRef = 255;      // max


byte noiseParameters[] = {
  7,   
  5,     
  3    
};

void setup() {
  
  PRR = (1 << PRUSI);                  // disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); // PB2,PB3  //disable digital input in pins that do analog conversion

  // set the rest of the pins
  pinMode(0, OUTPUT); // LED
  pinMode(4, OUTPUT); // timer 1 in digital output 4 - outs 1 and 2
  pinMode(3, INPUT);  // analog- freq input (knob plus external input 1)
  pinMode(2, INPUT);  // analog- amplitude input (external input 2)
  pinMode(1, INPUT);  // digital input (push button)
  
  checkVoltage();
  ADMUX = 0;                      // reset multiplexer settings

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
  TCCR0A = (1<<WGM01);                 // Clear Timer on Compare (CTC) with OCR0A
  TCCR0B = (1<<CS01) ;                 // prescale by 8
  OCR0A = 0;                           // frequencies down to 3921hz for a value of 255 https://www.easycalculation.com/engineering/electrical/avr-timer-calculator.php
  TIMSK = (1 << OCIE0A);               // Enable Interrupt on compare with OCR0A

  //Pin Change Interrupt
  GIMSK |= (1 << PCIE);    // Enable 
  PCMSK |= (1 << PCINT1);  // on pin 1
  
  initSeed(500);                       //initialize step information. 500 is the memory address for the random Seed

  sei();                               // Interrupts ON (enable interrupts globally)
  
  //go for it!
  digitalWrite(0, HIGH);               // turn LED ON
  
}

ISR(TIMER0_COMPA_vect) {               // Timer 0 interruption - changes the width of timer 1's pulse to generate waves

  OCR1B = xorshift32();
  
}

ISR(PCINT0_vect) {                       // PIN Interruption - has priority over COMPA; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;        // Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
  if (inputButtonValue) {
    controlFrequency = !controlFrequency;
    if (controlFrequency)flashLEDOnce();
    else flashLEDTwice();
  }
}

void loop() {
  
  randomizeNoise();   //randomizes the noise but you can still control freq and density
  //randomizeMore();  //randomizes the noise, frequency, and density - no user control 
}

byte xorshift32(void) {
    y32 ^= (y32 << noiseParameters[0]);
    y32 ^= (y32 >> noiseParameters[1]);
    return y32 ^= (y32 << noiseParameters[2]);  //pattern values: 8 bit
}

void randomizeMore() {
  
  randomizeNoiseParams();
  
  OCR0A = (rand() >> 7);    //frequency  - rand (max value 32767) >> 7 to give max 255
  OCR1C = (rand() >> 7);  //density - if used together with the delay below, it freezes the sound
  //delay(100);               //may freeze the sound depending on the value (and/or chance!) :P
}

void randomizeNoise() {               
  if (controlFrequency)setFrequency(3);
  else setGrainDensity(3);
  randomizeNoiseParams();
}

void randomizeNoiseParams() {
  noiseParameters[0] = (rand() >> 10) + 1;  //rand (max value 32767) >> 10 to give max 32
  noiseParameters[1] = (rand() >> 10) + 1;  // +1 to avoid 0, which might "extiguish" the noise
  noiseParameters[2] = (rand() >> 10) + 1;
}

//Parameters don't change until we return to the value they had last time we changed them.
//we store the knob's position in a variable and check the current position against it;
//when we reach it, we start controlling the parameter again.

void setGrainDensity(int pin) { 
  coarseFreqChange = false;                         // reset the control condition for frequency
  if (coarseGrainChange == false) {
    byte coarseGrainRead = analogRead(pin) >> 2; 
    if (coarseGrainRead == potPosGrainRef) {
      coarseGrainChange = true;
    }
  }
  if (coarseGrainChange == true) {
    int tempRead = analogRead(pin);
    byte densityRead = tempRead >> 2;                 // right shifting by 2 to get values between 0 and 255 (0-1023/2^2)
    potPosGrainRef =  densityRead;                    // save the knob´s position for reference.
    OCR1C = densityRead;
  }
}

void setFrequency(int pin) {
  coarseGrainChange = false;                           // reset the control condition for density
  if (coarseFreqChange == false) {
    byte coarsefreqRead = analogRead(pin) >> 2;
    if (coarsefreqRead == potPosFreqRef) {
      coarseFreqChange = true;
    }
  }
  if (coarseFreqChange == true) {
    int tempRead = analogRead(pin);
    byte freqRead = tempRead >> 2;
    potPosFreqRef = freqRead;
    OCR0A = 255 - freqRead;                          // reversing values so that the knob affects it in the same way as the other parameters                          
  }
}


void checkVoltage() {                   // voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5
                                        // we measure a fixed value of 1.1 against Vcc, so the lower the measurement, the higher Vcc
  ADMUX = (0 << REFS1)|(0 << REFS0);    // Vcc as reference
  ADMUX |= (1 << ADLAR);                // Left adjust result (8 bit conversion stored in ADCH)
  ADMUX |= (1 << MUX3) | (1 << MUX2);   // 1.1v input
  delay(250);                           // Wait for Vref to settle
  ADCSRA |= (1 << ADSC);                // Start conversion
  while (bit_is_set(ADCSRA, ADSC));     // wait while measuring
  if (ADCH > 103)                       // approx 2.6
    flashLED(8, 100);
  else
    flashLED(1, 250);
}

void flashLED (int times, int gap) {     // for voltage check only (uses regular delay)
  for (int i = 0; i < times; i++)
  {
    digitalWrite(0, HIGH);
    delay(gap);
    digitalWrite(0, LOW);
    delay(gap);
  }
}

void flashLEDOnce () {
  digitalWrite(0, LOW);
  _delay_ms(100);
  digitalWrite(0, HIGH);
}

void flashLEDTwice () {
  digitalWrite(0, LOW);
  _delay_ms(80);
  digitalWrite(0, HIGH);
  _delay_ms(80);
  digitalWrite(0, LOW);
  _delay_ms(80);
  digitalWrite(0, HIGH);
}


void initSeed(int address){  //initialize steps' info to random notes
  unsigned int rSeed = eeprom_read_word((uint16_t*)address);  //read the last seed we used, from memory
  rSeed++;                                                    //add one -now we have a new seed
  srand(rSeed);                                               //assign the new seed to the random generator (this way we'll have a new sequence of random values)
  eeprom_update_word((uint16_t*)address, rSeed);              //save the seed we used
}

