/*
//********************
//*    miniMO DCO    *
//* 2016 by enveloop *
//********************
//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license:
   http://creativecommons.org/licenses/by/4.0/
//
I/O
  1&2: Outputs - waveform
  3: Input - frequency modulation
  4: Input - amplitude modulation
  
MODES OF OPERATION
  PLAY (default)
    -Knob: change frequency (default) or amplitude
      -When you change between modifying frequency and amplitude, miniMO memorizes the point where you leave the knob 
      -When you start modifying a parameter again, miniMO waits until you reach the value where you left it earlier
    -Single click: cycle through the available waves, from less to more harmonics
      -After the saw wave there is a "silent wave" that mutes the output (stored volume remains unaffected)
      -After a single click, the LED blinks once
    -Double click: cycle through the frequency ranges
      -After a double click, the LED blinks twice
    -Triple click: go to frequency calibration mode (see below)
    -Click and hold: modify the amplitude using the knob 
      -While holding, the LED blinks constantly
      
  FREQUENCY CALIBRATION
  When you enter this mode, miniMO starts an automatic procedure and maps its frequency inputs to the expected ranges 
    -If you are connecting a miniMO sequencer or tuned controller: 
      -Move the knob all the way down
      -Set the frequency range to the middle
      -Click the button three times
        -The LED turns OFF
      -Sweep the input through the maximum and minimum values
      -Calibration finishes automatically if no new max or min values are registered for two seconds
        -The LED turns ON
    -If you disconnected an external source:
      -Click the button three times
        -The LED turns OFF
      -Move the knob all the way up and down a few times
      -The LED turns ON
      
  miniMO automatically saves the calibrated values to memory and recalls them if you turn it OFF and ON again
  
  CALIBRATION TROUBLESHOOTING
  
  Problem: When connected to an external device, calibration is lost after turning the OSC OFF and ON again
    -Solution: Disconnect any cable connected to the input 2 before turning it OFF
               Alternatively, turn OFF any device connected to the OSC input 2,
               or make sure that said device is not sending any signal to OSC's input 2 before turning the OSC OFF

  BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
*/

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

//button interrupt
volatile bool inputButtonValue;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
int additionalClicks = 0;      //variable to see how many times we click after the first

//freq control
bool coarseFreqChange = true;  //allow to change frequency on power up
byte potPosFreqRef = 255;      //max
int freqRange;
int freqRangeMin, freqRangeMax;

//freq calibration
int sensorValue = 0;
int sensorMax;
int sensorMin;
bool calibrating = false;

//volume control
bool coarseVolChange = false;
byte potPosVolRef = 7;          //max

//wave
int currentWave;

//output
byte volumeRead;     //pin reading (knob)
byte volume;         //pin reading and external modulation
volatile unsigned int frequency;  //pin reading (knob and external modulation). It's only one variable because the knob and external input share the input pin

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
  //disable USI to save power as we are not using it
  PRR = 1<<PRUSI;
  
  //set LED pin and check the battery level
  pinMode(0, OUTPUT); //LED
  checkVoltage();
  ADMUX = 0;                      //reset multiplexer settings

  //read calibrated values for freq input
  sensorMin = eeprom_read_word((uint16_t*)1);
  sensorMax = eeprom_read_word((uint16_t*)3);
  if (sensorMax == 0) sensorMax = 1023; //if there was no data in memory, give it the default value

  //set the rest of the pins
  pinMode(4, OUTPUT); //timer 1 in digital output 4 - outs 1 and 2
  pinMode(3, INPUT);  //analog- freq input (knob plus external input 1)
  pinMode(2, INPUT);  //analog- amplitude input (external input 2)
  pinMode(1, INPUT);  //digital input (push button)

  //disable digital input in pins that do analog conversion
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3
  
  //set clock source for PWM -datasheet p94
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1

  TIMSK  = 0;                          // Timer interrupts OFF

  //PWM Generation -timer 1
  GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb1, compare with OCR1B (see interrupt below), reset on match with OCR1C
  OCR1C  = 0xff;
  TCCR1  = (1 << CS10);                // no prescale

  //Timer Interrupt Generation -timer 0
  TCCR0A = (1 << WGM01) | (1 << WGM00); // fast PWM
  TCCR0B = (1 << CS00);                // no prescale
  TIMSK = (1 << TOIE0);                // Enable Interrupt on overflow

  //Pin interrupt Generation
  GIMSK |= (1 << PCIE);                // Enable Pin Change Interrupt
  PCMSK |= (1 << PCINT1);              // on pin 1

  sei();                               // Timer interrupts ON

  //default frequency and volume
  freqRange = 1;
  getMappedFreq(1);
  frequency = 1830;
  volumeRead = 255;
  volumeModulation = 255;

  //go for it!
  digitalWrite(0, HIGH);               // turn LED ON
  writeWave(0);                        // write a sine wave to the table
}

ISR(TIMER0_OVF_vect) {               //Timer 0 interruption - changes the width of timer 1's pulse to generate waves
  static byte sample;
  static unsigned int phase;
  OCR1B = sample;
  sample  = ((wavetable[phase >> 8] * volume) >> 8);
  phase += frequency;                                 //phase accumulator
}

ISR(PCINT0_vect) {                       //PIN Interruption - has priority over COMPA; this ensures that the switch will work
  inputButtonValue = digitalRead(1);
}

void loop() {
  checkButton();
  if (!calibrating) {
    setFrequency(3);                                 //change frequency 
    readExtInput(1);                                 //read analog input 1 (attiny PB2)
    volume = (volumeRead * volumeModulation) >> 8;   //modulate volume
  }
}

void calibrate() {
  calibrating = true;
  TIMSK = (0 << TOIE0);
  int firstRead = analogRead(3);
  int delta = 0;
  int sensorValue = 0;
  sensorMax = 0;                              //reset the max value
  sensorMin = 1023;                           //reset the min value
  int noNewMinOrMax = 0;
  while ( delta < 5) {                       //do nothing until there is "movement"
    sensorValue = analogRead(3);
    delta = abs(sensorValue - firstRead);
  }
  while (noNewMinOrMax < 200) {
    sensorValue = analogRead(3);
    if (sensorValue > sensorMax)sensorMax = sensorValue;       //input bigger than last - set new max
    else if (sensorValue < sensorMin)sensorMin = sensorValue;  //input smaller than last - set new min
    else noNewMinOrMax ++;                                  //input between min and max - nothing new
    _delay_ms(10);
  }
  eeprom_update_word((uint16_t*)1, sensorMin);
  eeprom_update_word((uint16_t*)3, sensorMax);
  calibrating = false;
  TIMSK = (1 << TOIE0);
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    button_delay++;
    _delay_ms(10);
    if (button_delay > 10 & ! beenDoubleClicked) {
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
        bool previousButtonState = inputButtonValue; //see if the button is pressed or not

        _delay_ms(1);

        button_delay_b++;                                               //fast counter to check if there are more presses
        if ((inputButtonValue == HIGH) && (previousButtonState == 0)) {
          additionalClicks++;                                                     //if we press the button and we were not pressing it before, that counts as a click
        }

        if (button_delay_b == 100) {
          if (additionalClicks == 0) {
            if (beenLongPressed) {                    //button released after being pressed for a while (most likely because we were changing the volume)
              beenLongPressed = false;
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
            else {                                    //button released (regular single click)
              flashLEDOnce();
              currentWave++;                           //change the wave
              if (currentWave >= 5) currentWave = 0;
              writeWave(currentWave);
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
          }
          else if (additionalClicks == 1) {             //button pressed again (double click),
            flashLEDTwice();
            freqRange++;                               //change the frequency range
            if (freqRange >= 3) freqRange = 0;
            getMappedFreq(freqRange);
            coarseFreqChange = true ;
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
          }
          else if (additionalClicks > 1 ) {                 //button pressed at least twice (triple click or more)
            digitalWrite(0, LOW);
            calibrate();
            digitalWrite(0, HIGH);
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

//We want parameters to change only if we return to the vaue where we left them after controlling something else
//so we store the knob's position in a variable and check the current position against it;
//when we reach it, we start controlling the parameter again.

void setVolume(int pin) {
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
    potPosVolRef = tempRead >> 7;                    //save the knob´s position for reference. Right shifting to get values between 0 and 7 (max resolution that works with the button pressed)
  }
}

void setFrequency(int pin) {
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
    frequency = map(tempRead, sensorMin, sensorMax, freqRangeMin, freqRangeMax); //map the calibrated values (by default 0-1023) to the frequency range we want
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
      freqRangeMax = 114;  //A1
      break;
    case 1:
      freqRangeMin = 114;
      freqRangeMax = 1830; //A5
      break;
    case 2:
      freqRangeMin = 1830;
      freqRangeMax = 3660; //A6
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

void checkVoltage() {                   //voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5
                                        //we measure a fixed value of 1.1 against Vcc, so the lower the measurement, the higher Vcc
  ADMUX |= (1 << ADLAR);                //Left adjust result (8 bit conversion stored in ADCH)
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

void flashLED (int times, int gap) {     //for voltage check only (uses regular delay)
  for (int i = 0; i < times; i++)
  {
    digitalWrite(0, HIGH);
    delay(gap);
    digitalWrite(0, LOW);
    delay(gap);
  }
}
