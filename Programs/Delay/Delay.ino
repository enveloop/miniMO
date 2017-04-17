/*
//************************
//*   miniMO DELAY       *
//*   2017 by enveloop   *
//************************
Based on the Arduino Audio Reverb
      by Martin Nawrath,
      as described Here:
http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-realtime-audio-processing/

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

WARNING! This effect can self-oscillate, producing very loud tones

I/O
  1&2 Outputs - delayed signal
  3: Input - parameter modulation
  4: Input - audio signal

OPERATION
  Knob: change feedback (default), buffer size or sample rate
    -miniMO waits until you reach the value it has currently stored
  Click: toggle between feedback, buffer size and delay time control
    -the LED blinks 1 to 3 times depending on the parameter selected (1-Feedback, 2-Buffer, 3-Rate)

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

 */

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 8000000

//interrupt variables accessed globally
volatile boolean gotReadings;
volatile byte controlInput;
volatile byte audioInput;
volatile bool inputButtonValue;

//button press control
int button_delay;

int iw1;
int iw2;

int index = 0;
byte delayBuffer[256];  // Audio Memory Array 8-Bit
byte sample;

bool parameterChange = false;
int currentParameter = 0;
byte parameters[] = {
  200,   //feedback
  255, //buffer length  
  0    //loop delay length
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
  
  sei();                               // Interrupts ON (enable interrupts globally)
  
  digitalWrite(0, HIGH);                    //lights on!
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
      gotReadings = true;          // readings are ready! this will trigger processing in the main loop, once every two measurings - so aprox 7.8 Khz
      ADMUX &= ~(1 << MUX1);       //select the audio input (ADC1, so MUX1 = 0 and MUX0=1. MUX0 was already set to 1 during setup) 
    }
    ADCSRA |=  (1<<ADSC);           // start next conversion
}

void loop() {                    
  while (!gotReadings)         // wait until we have readings 
                 
  gotReadings = false;            //as soon as we have readings, reset the condition
  
  checkButton();
  
  setParameter();
  
  sample = delayBuffer[index] ;   // read a sample from the delay buffer
  
  OCR1B = sample;                 // send sample to PWM Output
 
  iw1 = 127 - sample ;            // subtract offset from buffer sample
  iw2 = 127 - audioInput;         // subtract offset from new audio sample
  
  iw1 = (iw1 * parameters[0]) >> 8;  // scale delayed sample with potentiometer (originally iw * badc0 /255)
  
  iw2 = iw2 + iw1;                 // add delayed sample and new sample
  
  if (iw2 < -127) iw2 = -127;     // limiter 
  else if (iw2 > 127) iw2 = 127;  // limiter 

  sample = 127 + iw2;               // add offset to the result
              
  delayBuffer[index] = sample;     // store sample in delay buffer
  
  index++;
  
  variableDelay(parameters[2]);          //slow down the process with a delay
  
  if (index > parameters[1]) index = 0;  // aprox.30 ms for 256 samples (7.8Khz/256) 
} 

void variableDelay(int us) {
  while(us--)                    //http://stackoverflow.com/questions/32649032/what-does-whilex-mean-in-c/32649606
  _delay_us(1);
}

void setParameter() {
  if (parameterChange) parameters[currentParameter] = controlInput;
  
  else if (controlInput == parameters[currentParameter]) {  //check control input against stored value. If the value is the same (because we have moved the knob to the last known position for that parameter),
      parameterChange = true;                               //it is ok to change the value :)
  }
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    button_delay++;
    _delay_ms(10);
  }
  if ((inputButtonValue == LOW) && (button_delay > 0)) {   //button released after a while (regular single click)
    currentParameter++;
    if (currentParameter > 2) currentParameter = 0; //3 parameters, indexes from 0 to 2
    flashLEDSlow(currentParameter + 1);             //parameter 0 - flash once, etc
    parameterChange = false;                        //reset parameter change condition so that after we press the button, the newly selected parameter won't immediately change (see setParameter)
    button_delay = 0;
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
    _delay_ms(100);
    digitalWrite(0, LOW);
    _delay_ms(100);
    digitalWrite(0, HIGH);
  }
}
