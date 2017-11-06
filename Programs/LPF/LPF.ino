/*
//******************************
//*   miniMO Low Pass Filter   *
//*   2017 by enveloop         *
//******************************
  Based on the Filter Algorithm 
        by Rohan Hill,
      as described Here:
https://beammyselfintothefuture.wordpress.com/2015/02/16/simple-c-code-for-resonant-lpf-hpf-filters-and-high-low-shelving-eqs/
  
//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

WARNING! This filter can self-oscillate, producing very loud tones 

I/O
  1&2 Outputs - filtered signal
  3: Input - frequency/resonance modulation
  4: Input - audio signal

OPERATION
  Knob: change frequency (default) or resonance
    -miniMO waits until you reach the value it has currently stored
  Click: toggle between frequency and resonance control
    -The LED blinks once -  frequency control
    -The LED blinks twice - resonance control

BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
  
*/

#include <util/delay.h>

//button input interrupt
bool inputButtonValue;

//audio and control signal interrupt
byte audioInput, controlInput;
int count;
byte sensorMinDefault = 63;
byte sensorMin;

//control parameters
bool parameterChange = true; //behaviour after turning on: change cutoff frequency
int currentParameter = 0; 
byte parameters[] = {
  255,   //frequency (255: filteropen)
  0      //resonance
};

//button press control
int button_delay;

//LPF parameters
int lastOutput, output;
int momentum;

void setup() {
  
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion
  
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //audio output - outs 1 and 2
  pinMode(3, INPUT);  //analog- control input (knob plus external input 1)
  pinMode(2, INPUT);  //analog- audio input (external input 2)
  pinMode(1, INPUT);  //digital input (push button)
  
  checkVoltage();
  
  ADCSRA = (1<<ADEN);             //reset ADC Control (ADC Enable 1, everything else 0)
  ADCSRA |= (1<<ADPS2);           //set adc prescaler  to 16 for 500kHz sampling frequency (8 also works well but is noisier)
  
  ADMUX = 0;                                            //reset multiplexer settings
  //ADMUX |= (1 << REFS2) | (1 << REFS1);               //2.56V internal Voltage Reference disconnected from AREF
  ADMUX |= (0 << REFS2) | (0 << REFS1) | (0 << REFS0);  //Vcc as voltage reference --not necessary, but a reminder
  ADMUX |= (1<<ADLAR);                                  //left-adjust result (8 bit conversion) 
  ADMUX |= (1 << MUX0);                                 //select ADC1 (audio input)
  ADCSRA |= (1 << ADSC);                                //start conversion

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
  TCCR0A = (1 << WGM01) | (1 << WGM00);   // fast PWM
  TCCR0B = (1 << CS00);                   // no prescale
  TIMSK = (1 << TOIE0);                   // Enable Interrupt on overflow
  
  //Pin Change Interrupt
  GIMSK |= (1 << PCIE);    // Enable 
  PCMSK |= (1 << PCINT1);  // on pin 1

  sei();                               // Interrupts ON (enable interrupts globally)
  digitalWrite(0, HIGH);
}

ISR(PCINT0_vect) {                       //PIN Interruption - has priority over Timer 0; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;        //Reads button (digital input 1, the second bit in register PINB. We check the value with & binary 10, so 0x02)
}

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
        sensorMin = sensorMinDefault;                                //reset sensorMin
        ADMUX &= ~(1 << MUX1);                                       //select the audio input (ADC1, so MUX1 = 0 and MUX0=1. MUX0 was already set to 1 during setup) 
    }
 
    ADCSRA |=  (1<<ADSC);                                           //start next conversion
}

void loop() {
  
  OCR1B = doResonantLPF(audioInput);
  
  setParameter();
  checkButton();
}

void setParameter() {                                                //wait until we reach the latest value stored for that parameter, then start controlling it again
  if((controlInput >> 4) == (parameters[currentParameter] >> 4)){    //>>4, so we track 15 reference positions. This makes it easier to pick it up.   
      parameterChange = true;
   }
   if(parameterChange){ 
     parameters[currentParameter] = controlInput;
   }
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    button_delay++;
    _delay_ms(10);
  }
  if ((inputButtonValue == LOW) && (button_delay > 0)) {   //button released after a while (regular single click)
    currentParameter++;
    if (currentParameter > 1) currentParameter = 0; //2 parameters, indexes 0 and 1
    flashLEDSlow(currentParameter + 1);             //parameter 0 - flash once, etc
    parameterChange = false;
    button_delay = 0;
   }    
}

int doResonantLPF(int input) {
    
    input = input << 3;
    int distanceToGo = input - lastOutput;
    momentum = momentum + distanceToGo;     
    lastOutput = lastOutput + scale(momentum, parameters[1]) + scale(distanceToGo, parameters[0]);
    
    output = ((lastOutput) >> 3) + (parameters[1] >> 2);  

    if (output < 0) { output = 0; momentum = momentum >> 1;}
    if (output > 255) {output = 255; momentum = momentum >> 1;}
    
    return output;
}

int scale(int input, byte factor) {

  int output =                                              //every line adds precision - and output noise
            ( (input>>1) & (-((128&factor)>>7)) )         
            + ( (input>>2) & (-((64&factor)>>6)) )          
            + ( (input>>3) & (-((32&factor)>>5)) )          
            + ( (input>>4) & (-((16&factor)>>4)) )        
            + ( (input>>5) & (-((8&factor)>>3)) )  ; 
         /* + ( (input>>6) & (-((4&factor)>>2)) ) 
            + ( (input>>7) & (-((2&factor)>>1)) )
            + ( (input>>8) & (- (1&factor)    ) ); */
  
  return output;
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
