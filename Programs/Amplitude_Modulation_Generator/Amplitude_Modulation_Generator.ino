/*
//*************************************************
//*   miniMO AMPLITUDE MODULATION GENERATOR       *
//*             2017 by enveloop                  *
//*************************************************
Based on the Arduino Ring Modulator
      by Martin Nawrath,
      as described Here:
http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-realtime-audio-processing/

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

DESCRIPTION

This program modulates the amplitude of a wave, the carrier, with a second wave, the modulator 
Both waves are stored internally and have the same shape
You have control over their frequencies, shape, and kind of modulation
You can further modulate their frequencies using the external inputs

I/O
  1&2: Outputs - processed signal
  3: Input - modulates currently selected parameter
  4: Input - always modulates the frequency of the modulator wave

OPERATION
  Knob: change carrier frequency (default) or modulator frequency
    -When you change between parameters, miniMO memorizes the point where you leave the knob 
    -When you start modifying a parameter again, miniMO waits until you reach the value where you left it earlier
  Click: toggle between carrier or modulator frequency
    -The LED blinks once
  Double Click: cycle through the available wave shapes
    -The LED blinks twice
    -Shapes (in order): sine, triangle, square, and saw
    -Note: Both modulator and carrier have the same wave shape
  Click, hold, and release after a couple of seconds: cycle through the available modulations
    -The LED blinks thrice
    -Modulation types (in order): Ring Modulation, Amplitude Modulation, and Amplitude Modulation Plus

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

NOTES:

  ON THE TYPES OF MODULATION
    When both the Carrier and Modulator are sine Waves of frequencies F1 and F2 respectively, 
      -Ring Modulation produces two new frequencies, F2 + F1, and F2 - F1
      -Amplitude Modulation preserves F2 and also produces F2 + F1 and F2 - F1
      -Amplitude Modulation Plus preserves both F1 and F2, and it also produces F2 + F1 and F2 - F1

  ON EXTERNAL MODULATION
     If you are controlling the frequency of the modulator wave with the knob,
      -A signal applied to I/O 3 will change the frequency, with the knob setting the central value (offset to both the maximum and minimum values)
      -A signal applied to I/O 4 will change the frequency, with the knob setting the maximum value 
 */

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 8000000

//interrupt variables accessed globally
volatile bool gotReadings;
volatile byte controlInput;
volatile byte audioInput;
volatile bool inputButtonValue;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
byte additionalClicks = 0;

byte currentWave;
byte currentModulation;

int iw1, iw2;

//table for half a cycle of the sine wave (will be mirrored to make the other half)
const char PROGMEM sinetable[128] = {
  0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35, 37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};

//the actual table that is read
unsigned char wavetable[256];

static byte sample, sample2;
static unsigned int phase, phase2;

bool parameterChange = true;
int currentParameter = 1;      //start program modulating carrier
byte parameters[] = {
  20, //freq wave 1 (modulator)
  80, //freq wave 2 (carrier)
};

void setup() {
  
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion
  
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //timer 1 in digital output 4 - outs 1 and 2
  pinMode(3, INPUT);  //analog- freq input (knob plus external input 1)
  pinMode(2, INPUT);  //analog- amplitude input (external input 2)
  pinMode(1, INPUT);  //digital input (push button)
  
  checkVoltage();
  
  ADCSRA = (1 << ADEN);             //reset ADC Control (ADC Enable 1, everything else 0)
  ADCSRA |= (1 << ADPS2);           //set adc prescaler  to 16 for 500kHz sampling frequency (8 also works well but is noisier). 500/13 cycles per  sample = 38.4 Khz, faster than the timer interrupt -good!
  
  ADMUX = 0;                              //reset multiplexer settings
  //ADMUX |= (1 << REFS2) | (1 << REFS1);               //2.56V internal Voltage Reference disconnected from AREF
  ADMUX |= (0 << REFS2) | (0 << REFS1) | (0 << REFS0);  //Vcc as voltage reference --not necessary, but a reminder
  ADMUX |= (1 << ADLAR);                    //8-Bit ADC in ADCH Register
  ADMUX |= (1 << MUX0);                   //select ADC1 (audio input)
  ADCSRA |= (1 << ADSC);                    // start conversion
  
  //set clock source for PWM -datasheet p94
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock: do nothing while the bit PLOCK in register PLLCSR is false
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1

  cli();                               // Interrupts OFF (disable interrupts globally)

  //PWM Generation -timer 1
  GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb1, compare with OCR1B (see interrupt below), reset on match with OCR1C
  OCR1C  = 0xff;
  TCCR1  = (1 << CS10);                  // no prescale

  //Timer Interrupt Generation -timer 0
  TCCR0A = (1 << WGM01) | (1 << WGM00);    // fast PWM
  TCCR0B = (1 << CS00);                    // no prescale (source: internal clock)
  TIMSK = (1 << TOIE0);                    // Enable Interrupt on overflow, triggered at 31.2KHz (8Mhz / 256 steps per overflow)

  //Pin interrupt Generation
  GIMSK |= (1 << PCIE);                  // Enable Pin Change Interrupt
  PCMSK |= (1 << PCINT1);                // on pin 1
  
  sei();                                 // Interrupts ON (enable interrupts globally)
  
  writeWave(0);                              // fills the wave table with the sine wave stored in progmem
  
  digitalWrite(0, HIGH);                   // lights on!
}

ISR(PCINT0_vect) {                 //PIN Interruption - has priority over Timer 0; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;  //Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

//Timer0 interruption
ISR(TIMER0_OVF_vect) {           //alternates between reading audio and control input, so each channel is sampled at 15.6kHz
    
    if (!(ADMUX & 0x02)){         //if the audio input is selected... (it's ADC1, so MUX1 = 0. MUX1 is the second bit in register ADMUX; then, ADMUX & binary 10 = 0, or !(ADMUX&0x02). That ! is logic, so it's not inverting anything, but checking that the value is false       
      audioInput = ADCH;          // read the value 
      ADMUX |= (1 << MUX1);       //select the control input (ADC3, so MUX1 = 1 and MUX0 = 1. MUX0 was already set to 1 during setup) 
    }
    else
    {
      controlInput = ADCH;         // read the value
      gotReadings = true;
      ADMUX &= ~(1 << MUX1);       //select the audio input (ADC1, so MUX1 = 0 and MUX0=1. MUX0 was already set to 1 during setup) 
    }
    ADCSRA |=  (1<<ADSC);           // start next conversion
}

void loop() {   
    while (!gotReadings) {         // wait until we have readings 
  }                  
  gotReadings = false;  
  setParameter();
  checkButton();  
  doModulation(currentModulation);
} 

void setParameter() {
  if (parameterChange) parameters[currentParameter] = controlInput;
  
  else if (controlInput == parameters[currentParameter]) {  //check control input against stored value. If the value is the same (because we have moved the knob to the last known position for that parameter),
      parameterChange = true;                               //it is ok to change the value :)
  }
}

void checkButton() {
   while (inputButtonValue == 1) {
     button_delay++;
     _delay_ms(10);
     if (button_delay > 10 & ! beenDoubleClicked) {
       beenLongPressed = true;                      //press and hold
      }
    }
    if (inputButtonValue == 0) {
      beenDoubleClicked = false;
      if (button_delay > 0) {
        bool hold = true;
        while (hold) {
          bool previousButtonState = inputButtonValue; //see if the button is pressed or not
  
         _delay_ms(1);
         button_delay_b++;                                                  //fast counter to check if there are more presses
          
        if ((inputButtonValue == HIGH) && (previousButtonState == 0)) {   
          additionalClicks++;                                              //if we press the button and we were not pressing it before, that counts as a click
        }
          
        if (button_delay_b == 300) {
            
          if (additionalClicks == 0) {                //single click
            if (beenLongPressed) {                    //long press
               flashLEDSlow(3);
               currentModulation++;                   //change type of modulation
               if (currentModulation > 2) currentModulation = 0;   
              beenLongPressed = false;
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
              }
            else {                                    //still single click, but short press
              flashLEDSlow(1); 
              currentParameter++;                     //change parameter (in this case, there are two parameters: freq os wave 1, and freq of wave2)
              if (currentParameter > 1) currentParameter = 0; 
              parameterChange = false;            
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
          }   
          else if (additionalClicks >= 1) {              //two or more clicks
            flashLEDSlow(2); 
            currentWave++;                               //change the wave shape (both waves have the same shape -there's no space to store them separately)
            if (currentWave > 3) currentWave = 0;
            writeWave(currentWave); 
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

//////////////////////////
void doModulation(int mod) {
  switch (mod) {
    case 0:
      ringMod();
      break;
    case 1:
      ampMod();
      break;
    case 2:
      ampModPlus();
      break;
  }
}

//types of amplitude modulation
void ringMod() {                    //multiply both waves in range -127 to 127
  OCR1B = sample;
    sample = (wavetable[phase >> 8]); 
    sample2 = (wavetable[phase2 >> 8]); 
    iw2 = 127 - sample;
    iw1 = 127 - sample2;
    iw1 = (iw1 * iw2) >> 7;
    sample = iw1 + 127;
    phase += (audioInput * (parameters[0] >> 1)) >> 1;    //audioInput modulates the frequency of the modulator wave in a range defined by parameters[0] 
    phase2 += parameters[1] << 6;                         //this shift, and the last shift above, control the maximum frequency for both waves
}

void ampMod() {                     //multiply carrier in range -127 to 127 with modulator in range 0 to 127 
 OCR1B = sample;
    sample = (wavetable[phase >> 8]) >> 1; //0-127
    sample2 = (wavetable[phase2 >> 8]); 
    iw1 = 127 - sample2;
    iw1 = (iw1 * sample) >> 7;
    sample = iw1 + 127;
    phase += (audioInput * (parameters[0] >> 1)) >> 1;
    phase2 += parameters[1] << 6;
}

void ampModPlus() {                //multiply both waves in range 0-255
  OCR1B = sample;
    sample = (wavetable[phase >> 8]); 
    sample2 = (wavetable[phase2 >> 8]);
    sample = (sample * sample2) >> 8;
    phase += (audioInput * (parameters[0] >> 1)) >> 1;
    phase2 += parameters[1] << 6;
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
  }
}

//functions to populate the wavetable
void sineWave() {                                       //too costly to calculate on the fly, so it reads from the sine table. We use 128 values, then mirror them to get the whole cycle
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
  wavetable[128] = 255;
  for (int i = 129; i < 256; ++i) {
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

void checkVoltage() {                   //voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5
                                        //we measure a fixed value of 1.1 against Vcc, so the lower the measurement, the higher Vcc
  ADMUX = (0 << REFS1) | (0 << REFS0);  //Vcc as reference
  ADMUX |= (1 << ADLAR);                //Left adjust result (8 bit conversion stored in ADCH)
  ADMUX |= (1 << MUX3) | (1 << MUX2);   //1.1v input
  delay(250);                           //Wait for Vref to settle
  ADCSRA |= (1 << ADSC);                //Start conversion
  while (bit_is_set(ADCSRA, ADSC));     //wait while measuring
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

void flashLEDSlow(int times) {
  for (int i = 0; i < times; i++){
    _delay_ms(50);
    digitalWrite(0, LOW);
    _delay_ms(50);
    digitalWrite(0, HIGH);
  }
}
