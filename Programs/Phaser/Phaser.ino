/*
//************************
//*   miniMO PHASER      *
//*   2017 by enveloop   *
//************************
Based on the Arduino Audio Phasor
      by Martin Nawrath,
      as described Here:
http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-realtime-audio-processing/

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

I/O
  1&2 Outputs - phased signal
  3: Input - parameter modulation
  4: Input - audio signal

OPERATION
  Knob: change phase (default), phased buffer size or sample rate
    -miniMO waits until you reach the value it has currently stored
  Click: toggle between phase, phased buffer size and delay time control
    -the LED blinks 1 to 3 times depending on the parameter selected (1-Phase, 2-Buffer, 3-Rate)
  Double Click: toggle between normal and high sensitivity modes

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

 */

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 8000000

//audio and control signal interrupt
int count;
byte sensorMinDefault = 63;
byte sensorMin;

//interrupt variables accessed globally
volatile boolean gotReadings;
volatile byte controlInput;
volatile byte audioInput;
volatile bool inputButtonValue;

//button press control
int button_delay;
int button_delay_b;
int additionalClicks = 0;

int test;
int iw1, iw2;
int phaseSample;
int index, phaseIndex;
byte sampleBuffer[256];  // Audio Memory Array 8-Bit

bool highSensitivity = false;

bool parameterChange = false;
int currentParameter = 0;
byte parameters[] = {
  127, //phase
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

ISR(PCINT0_vect) {                 // PIN Interruption - has priority over Timer 0; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;  // Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

//Timer0 interruption
ISR(TIMER0_OVF_vect) {                                             //Alternates between reading the audio (most of the time) and control input.
                                                                                                                                                                                                
  if (!(ADMUX & 0x02)){                                             //if the audio input is selected... (it's ADC1, so MUX1 = 0. Then, ADMUX & binary 10 = 0, or !(ADMUX&0x02)  
      
      audioInput = ADCH;                                           //read the value 
  
      if (count < 100){                                             //until count is 100, 
         if (audioInput < sensorMin) sensorMin = audioInput;        //if the audio input is low, adjust the threshold
         count++;                                                   //advance the counter   
      }
      else {                                                        //from 101 onwards
          if (audioInput == sensorMin){                             //the moment the threshold is reached (makes the readings periodical),  
            ADMUX |= (1 << MUX1);                                   //select the control input (ADC3, so MUX1 = 1 and MUX0=1. MUX0 was already set to 1 during setup) 
            ADMUX &= ~(1 << REFS1);                                 //the control input always needs Vcc for reference to read the potentiometer's full range, so REFS1 is 0 
          }
          else if (count > 200) {                                   //if the threshold is not reached after a while,
            sensorMin = sensorMinDefault;                           //reset 
            count = 0;
          }
         else count++;
        }
    }
    else {                                                           //if the control input is selected
        controlInput = ADCH;                                         //read the value
        gotReadings = true;
        sensorMin = sensorMinDefault;                                //reset sensorMin
        ADMUX &= ~(1 << MUX1);                                       //select the audio input (ADC1, so MUX1 = 0 and MUX0=1. MUX0 was already set to 1 during setup) 
        if (highSensitivity) ADMUX |= (1 << REFS1);                  //if highSensitivity is selected, we toggle REFS1 to use 1.1v as voltage reference
    }
 
    ADCSRA |=  (1<<ADSC);                                           //start next conversion
}

void loop() {                    
  while (!gotReadings)             // wait until we have readings 
                 
  gotReadings = false;             // as soon as we have readings, reset the condition
  
  checkButton();
  
  setParameter();
  
  //retrieve stored sample    
  phaseSample = sampleBuffer[phaseIndex];    // retrieve sample
  phaseIndex = index - parameters[0];       // the control input sets the phase
  phaseIndex = phaseIndex & parameters[1];  // limit index
  
  OCR1B = (phaseSample + audioInput) >> 1;  // average newly acquired sample with phased sample, and send the result to the PWM output           

  //store newly acquired sample to use in the next cycle
  sampleBuffer[index] = audioInput;  
  index++;                            
  index = index & 255;                       // limit index

  variableDelay(parameters[2]);
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
  
  if ((inputButtonValue == LOW) && (button_delay > 0)) {   //button released after a while 
    bool hold = true;
    while (hold) {
      bool previousButtonState = inputButtonValue; //see if the button is pressed or not
        
        _delay_ms(1);
       
        button_delay_b++;                                                  //fast counter to check if there are more presses
        if ((inputButtonValue == HIGH) && (previousButtonState == 0)) {   
          additionalClicks++;                                              //if we press the button and we were not pressing it before, that counts as a click
    }
        
      if (button_delay_b == 300) {
          
        if (additionalClicks == 0) {  //single click
          currentParameter++;
          if (currentParameter > 2) currentParameter = 0; //3 parameters, indexes from 0 to 2
          flashLEDSlow(currentParameter + 1);             //parameter 0 - flash once, etc
          parameterChange = false;                        //reset parameter change condition so that after we press the button, the newly selected parameter won't immediately change (see setParameter)
          button_delay = 0;
          button_delay_b = 0;
          additionalClicks = 0;
          hold = false;
        }   
        else {                         // more than one click
          highSensitivity = !highSensitivity;
          //ADMUX ^= (1 << REFS1);     // toggles REFS1 from 0 to 1 or 1 to 0. When it is 0, reference voltage is Vcc. When it is 1, reference voltage is 1.1v, so the ADC is more sensitive. 
          flashLEDSlow(1);         
          button_delay = 0;
          button_delay_b = 0;
          additionalClicks = 0;
          hold = false;
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

