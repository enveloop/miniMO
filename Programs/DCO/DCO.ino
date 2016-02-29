/*
//********************
//*    miniMO DCO    *
//* 2016 by enveloop *
//********************
//
   http://www.envelooponline.com/minimo
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//
//Modes of Operation
//Default: the knob modifies the frequency.
//Single click: cycles through the available waves, from less to more harmonics.
//  -After the saw wave comes a "silent wave", for when one wants to quickly silence the unit without changing the volume or turning it off.
//  -After every click, the LED blinks once.
//Double click: cycles through the frequency ranges.
//  -After the double click, the LED blinks twice.
//Click and hold: the knob modifies the amplitude.
//  -Whilst in this mode, the LED blinks constantly.
//  -As you change between frequency and amplitude, miniMO memorizes the place where you leave the knob for each parameter.
//  -When you go back to modifying a parameter, miniMO won't respond until you reach the value where you left it earlier.
//Input 1: connect an external source for frequency modulation.
//Input 2: connect an external source for amplitude modulation.
*/

#include <avr/io.h>
#include <util/delay.h>

//button interrupt
volatile bool inputButtonValue;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;

//freq control
bool coarseFreqChange = true;
int freqRange;
int freqRangeMin, freqRangeMax;

//wave
int currentWave;

//volume control
bool coarseVolChange = false;

//output
byte volumeRead;     //pin reading (potentiometer)
byte volume;         //pin reading and external(ADC) modulation 
int frequency;       //pin reading (potentiometer). The modulating signal doesn't go through the ATtiny

//external input smoothing
const int numReadings = 4;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
byte volumeModulation = 0;

//table for half a cycle of the sine wave (will be mirrored to make the other half)
const char PROGMEM sinetable[128] = {
  0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};

//the actual table that is read to generate the sound
unsigned char wavetable[256];

void setup() {
  //set pins
  pinMode(4, OUTPUT); //timer 1 in digital output 4
  pinMode(3, INPUT);  //analog- freq input
  pinMode(2, INPUT);  //analog- amplitude input
  pinMode(1, INPUT);  //digital input
  pinMode(0, OUTPUT); //LED
  
  //disable digital input in pins that do analog conversion
  DIDR0 = (1<<ADC2D)| (1<<ADC3D); 
  
  //set clock source for PWM -datasheet p94
  PLLCSR |= (1<<PLLE);                 // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1<<PLOCK)));      // Ensure PLL lock
  PLLCSR |= (1<<PCKE);                 // Enable PLL as clock source for timer 1

  TIMSK  = 0;                          // Timer interrupts OFF
  
  //PWM Generation -timer 1
  GTCCR  = (1<<PWM1B)| (1<<COM1B1);    // PWM, output on pb1, compare with OCR1B (see interrupt below), reset on match with OCR1C 
  OCR1C  = 0xff;
  TCCR1  = (1<<CS10);                  // no prescale

  //Timer Interrupt Generation -timer 0                                                          
  TCCR0A = (1<<WGM01) | (1<<WGM00);    // fast PWM
  TCCR0B = (1<<CS00);                  // no prescale
  TIMSK = (1 << TOIE0);                // Enable Interrupt on overflow
  
  //Pin interrupt Generation
  GIMSK |= (1<<PCIE);                  // Enable Pin Change Interrupt 
  PCMSK |= (1<<PCINT1);                // on pin 01 
  
  sei();                               // Timer interrupts ON
  
  //default frequency and volume
  freqRange = 1;
  getMappedFreq(1);
  frequency = 800 ;
  volumeRead = 255;
  volumeModulation = 255;
  
  //go for it!
  digitalWrite(0, HIGH);               // turn LED on
  writeWave(0);                        // write a sine wave to the table
}


ISR(TIMER0_OVF_vect) {               //Timer 0 interruption - changes the width of timer 1's pulse to generate waves
  static byte sample;
  static unsigned int phase;
  OCR1B = sample;    
  sample  = ((wavetable[phase >> 8]*volume)>>8);
  phase += frequency;                                 //phase accumulator
}

ISR(PCINT0_vect) {                       //PIN Interruption - has priority over COMPA; this ensures that the switch will work
   inputButtonValue = digitalRead(1);
}

void loop() {  
  checkButton();
  setFrequency(3);                                 //change frequency
  readExtInput(1);      
  volume = (volumeRead * volumeModulation)>>8;     //modulate volume    
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    button_delay++;
    _delay_ms(10);
    if (button_delay > 10 &! beenDoubleClicked) {
      beenLongPressed = true;                      //press and hold 
      flashLEDOnce();
      setVolume(3);                                //change the volume
    }
  }
  if (inputButtonValue == LOW) {
    beenDoubleClicked = false;
    if (button_delay > 0) {
      bool hold = true;
      while (hold) {
        _delay_ms(1);
        button_delay_b++;                           //fast counter to check if there's a second press
        
        if (button_delay_b == 100) {                
          if (beenLongPressed) {                    //button released after being pressed for a while (most likely because we were changing the volume)
            beenLongPressed = false;
            button_delay = 0;
            button_delay_b = 0;
            hold = false;
          }
          else {                                    //button released (regular single click)
            flashLEDOnce();
            currentWave++;                           //change the wave
	    if (currentWave >= 5) currentWave = 0;         
	    writeWave(currentWave);     
            button_delay = 0;
            button_delay_b = 0;
            hold = false;
          }
        }
       
        else if (inputButtonValue == HIGH) {         //button pressed again (double click),         
          flashLEDTwice();
	  freqRange++;                               //change the frequency range 
          if (freqRange >= 3) freqRange = 0; 
          getMappedFreq(freqRange); 
          coarseFreqChange = true ; 
          button_delay = 0;
          button_delay_b = 0;
          beenDoubleClicked = true;
          hold = false;
        }
      }
    }
  }
}
 
//We want parameters to change only if we return to the vaue where we left them after controlling something else
//so we store the potentiometer's position in a variable and check the current position against it; 
//when we reach it, we start controlling the parameter again.

void setVolume(int pin) {
  static byte potPosVolRef;
  coarseFreqChange = false;   //reset the control condition for frequency
  if (coarseVolChange == false) {
    byte coarsevolRead = analogRead(pin) >> 7; //right shifting to get values between 0 and 7
    if (coarsevolRead == potPosVolRef) {
      coarseVolChange = true;
    }
  }
  if (coarseVolChange == true) {
    int tempRead = analogRead(pin);
    volumeRead = tempRead >> 2;                      //right shifting by 2 to get values between 0 and 255 (0-1023/2^2)
    volume = (volumeRead * volumeModulation) >> 8;
    potPosVolRef = tempRead >> 7;                    //save the potentiometer´s position as reference. Right shifting to get values between 0 and 7
  }                                   
}

void setFrequency(int pin) {
  static byte potPosFreqRef;
  coarseVolChange = false;                            //reset the control condition for volume
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
    frequency = map(freqRead, 0, 255, freqRangeMin, freqRangeMax);
  }                            
}

void writeWave(int wave) {                    
  switch (wave) {
    case 0:
      sineWave();    
      break;
    case 1:
      triangleWave();
      break;
    case 2:
      squareWave();
      break;
    case 3:
      sawtoothWave();
      break;
    case 4:
      digitalWrite(0, LOW);
      zeroWave();
      break;
  }
}

//functions to populate the wavetable
void sineWave() {                                       //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i); 
  }
  wavetable[128] = 255;
  for (int i = 129; i < 255; ++i) {
    wavetable[i] = wavetable[256 - i] ;
  }
}
void sawtoothWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = i; // sawtooth
  }
}
void triangleWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = value;
    value -= 2;
  }
}
void squareWave() {
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems (offset and different freq), related to sample  = ((wavetable[phase >> 8]*amplitude)>>8);
  }
}
void zeroWave() {
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = 1;                  //0 gives problems
  }
}

int readExtInput(const byte pin) {  //with averaging
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(pin) >> 2; //right shifting by 2 to get values between 0 and 255 (0-1023/2^2)
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) readIndex = 0;
  volumeModulation = total / numReadings;                    
}

void getMappedFreq(int range) {                               
  switch (range) {
    case 0:
      freqRangeMin = 1;
      freqRangeMax = 100;
      break;
    case 1:
      freqRangeMin = 100;
      freqRangeMax = 1500;
      break;
    case 2:
      freqRangeMin = 1500;
      freqRangeMax = 3000;
      break;
  }
}

void flashLEDOnce () {
  digitalWrite(0, LOW);                  
  _delay_ms(20);
  digitalWrite(0, HIGH);
}

void flashLEDTwice () {
  digitalWrite(0, LOW);                 
  _delay_ms(20);
  digitalWrite(0, HIGH);
  _delay_ms(20);
  digitalWrite(0, LOW); 
  _delay_ms(20);
  digitalWrite(0, HIGH);
}
  
  
