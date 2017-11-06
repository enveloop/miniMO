/*
//**********************
//*  miniMO LUNAMOD    *
//* 2017 by enveloop   *
//**********************
  Based on a Remix by Rob Miles, shown here, http://hosted.hackaday.com/lunaMod45remix.pde
  of the original project by Brian McNamara: http://makezine.com/projects/make-26/the-luna-mod-looper/

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license:
   http://creativecommons.org/licenses/by/4.0/
//

CONCEPT

This program plays a series of 64 notes in a loop. 
While you keep the button pressed, it records the knob's positions as notes in the loop. 
Depending of the tempo and the time you keep the button pressed, you can modify individual notes, or the entire loop at once. 

I/O
  1&2: Outputs - signal
  3: Input - frequency/tempo modulation
  4: Input - wave change

OPERATION

  Knob: change tempo (default) or frequency
    -If you change tempo, miniMO waits until you reach the value it has currently stored
    -The LED toggles ON/OFF every 8 notes to give an indication of the tempo
  Button Press: record the knob's position
    -Recording continues for as long as the button is pressed
    -After the button is depressed, the knob controls tempo once more
  Finger Tap on both terminals of I/O 4: cycle through the available wave shapes
    -Shapes (in order): triangle, square, and saw 
    -You can also use an external source to change waves automatically

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

NOTES:

  ON CHANGING THE WAVE
    This program uses I/O 4 as a makeshift extra button.
    The best way to try this feature is to set the tempo to a low value, then gently place a finger over BOTH pins of I/O 4, until the wave changes.
    Usually it takes pressing for about a "LED interval" for the wave to change.
    This is an experimental feature! if for whatever reason it doesn't work for you, please let me know
    To disable this feature,
      comment the line that has the following code: if (analogRead(1) < 700) advanceWave();
      modify writeWave(value from 0 to 2) to set the initial wave shape.
*/

#include <util/delay.h>

//tone sequence
int currentStep = 0;                                              
int steps[] = {500, 500, 100, 100, 100, 100, 100, 100,             //Initial array of tones. The values are translated to frequencies (warning: "100" is not 100 HZ!)
               100, 100, 100, 100, 100, 100, 100, 100,             
               500, 500, 100, 100, 100, 100, 100, 100,             //The values here determine the melody that plays when you turn the unit ON
               100, 100, 100, 100, 100, 100, 100, 100,
               500, 500, 100, 100, 100, 100, 100, 100,
               100, 100, 100, 100, 100, 100, 100, 100,
               500, 500, 100, 100, 100, 100, 100, 100,
               100, 100, 100, 100, 100, 100, 100, 100
              };
              
//sound generation                               
volatile unsigned int frequency;
unsigned char wavetable[256]; //the table that stores the shape of the waves for PWM sound generation
int currentWave;

//tempo control 
int tempoReading;
byte tempoCoarse;
int tempo = 2000;             //max value for tempo = 1023 << 2 (see setTempo below)
bool tempoChange = false;

void setup()                                        
{
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion
  
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //timer 1 in digital output 4 - outs 1 and 2
  pinMode(3, INPUT);  //analog- tempo/freq input (knob plus external input 1)
  pinMode(2, INPUT);  //analog- tap input (experimental)
  pinMode(1, INPUT);  //digital input (push button)
  
  checkVoltage();
  ADMUX = 0;                      //reset multiplexer settings
  
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
  TCCR0A = (1 << WGM01) | (1 << WGM00); // fast PWM
  TCCR0B = (1 << CS00);                 // no prescale
  TIMSK = (1 << TOIE0);                 // Enable Interrupt on overflow
  
  sei();                                // Interrupts ON (enable interrupts globally)
  
  writeWave(0);    

  delay(40000);                         // delay to help tell when the LED blinking from battery reading stops
}

ISR(TIMER0_OVF_vect) {                   //Timer 0 interruption - changes the width of timer 1's pulse to generate waves
  static byte sample;
  static unsigned int phase;
  OCR1B = sample;
  sample = wavetable[phase >> 8];
  phase += frequency;                    //phase accumulator
}

void loop() {
  for (int i = 0; i < 64; i++) {                         //reads through the note array
    currentStep = i;                                
    if (i == 0 || i == 15 || i == 31 || i == 47) {       
      digitalWrite(0, HIGH);                             //turn the LED ON every 16 notes                          
    }
    else if (i == 7 || i == 23 || i == 39 || i == 55) {   
      digitalWrite(0, LOW);                              //turn the LED OFF every 16 notes, with an offset of 8 (combined with the code above, toggles the LED every 8 notes)
      if (analogRead(1) < 700) advanceWave();            //detect finger tap in I/O 4 -experimental! if doesn't work for you, try different values or comment the line
    }
    if (digitalRead(1) == HIGH) {                          //if the button is being pressed
      tempoChange = false;                                 //reset the tempo change condition
      steps[currentStep] = (analogRead(3) << 4);           //write the knob's position in the note array
      frequency = steps[currentStep];                                              
    }
    else {                                                 //if the button is not pressed
      setTempo();                                          //change the tempo (see below)
      frequency = steps[currentStep];                      //change the frequency according to the current step
    }
    delay(4092-tempo);                                   //wait a bit. 4092 is the maximum value the delay can take (see setTempo). The expression 4092-tempo nakes the positions that give the lowest notes also give the lowest tempos
  }
}

//We want the tempo to change only if we return to the vaue where we left it after controlling something else
//so we store the knob's position in a variable and check the current position against it;
//when we reach it, we start controlling the parameter again.

void setTempo() {
  if (tempoChange) {
    tempoReading = analogRead(3); 
    tempoCoarse = tempoReading >> 4;                //right shifting by 4 to get values between 0 and 63 (0-1023/2^4). Less resolution makes it easier to track
    tempo = tempoReading << 2;                      //Here we decide what's the actual tempo.The max value is 1023 << 2      
  }
  else if ((analogRead(3) >> 4) == tempoCoarse) {    //check control input against stored value. If the value is the same (because we have moved the knob to the last known position for that parameter),
      tempoChange = true;                            //it is ok to change the value :)
  }
}

void advanceWave()
{
 currentWave++;                           //change the wave
 if (currentWave >= 3) currentWave = 0;
 writeWave(currentWave);
}

void writeWave(int wave) {
  switch (wave) {
    case 0:
      triangleWave();
      break;
    case 1:
      squareWave();
      break;
    case 2:
      sawtoothWave();
      break;
  }
}

//functions to populate the wavetable
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
