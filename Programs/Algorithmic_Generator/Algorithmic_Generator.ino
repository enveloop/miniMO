/*
//**********************************
//*  miniMO Algorithmic Generator  *
//*       2018 by enveloop         *
//**********************************

Based on the RCArduino Algorithmic Music code by Duane Banks, 
 as described Here:
http://rcarduino.blogspot.com.es/2012/09/algorithmic-music-on-arduino.html

, and the Algorithmic Noise Machine, by Madshobye:
https://www.instructables.com/id/Algorithmic-noise-machine/

Also uses the xorshift pseudorandom number generator,
 as described Here:
http://www.arklyffe.com/main/2010/08/29/xorshift-pseudorandom-number-generator/ 
 
I/O
  1&2: Outputs 
  3: Input - parameter modulation
  4: Input - unused
  
OPERATION
  Knob: change the value of the parameter currently selected
    -miniMO waits until you reach the value it has currently stored 
  Click: toggle between parameters
    -There are three parameters available per algorithm 
    -The LED blinks once, twice, or thrice, according to the parameter selected 
  Double Click: toggle between algorithms
    -The LED blinks once
        
BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low

NOTES&TROUBLESHOOTING
  Sometimes the sound changes when moving between parameters 


//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license:
   http://creativecommons.org/licenses/by/4.0/
//

*/

#include <avr/io.h>
#include <util/delay.h>

char sample;
long t;
byte controlInput;
bool parameterChange = false;
int currentParameter = 0;
int currentAlgo = 0;
byte parameters[] = {
  1,   
  1,     
  1    
};

//button press control
int button_delay;
int button_delay_b;
int additionalClicks = 0;

//button interrupt
bool inputButtonValue;

void setup() {
  
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion

  //set the rest of the pins
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //
  pinMode(3, INPUT);  //analog- parameter input (knob plus external input 1)
  pinMode(2, INPUT);  //analog-
  pinMode(1, INPUT);  //digital input (push button)
  
  checkVoltage();
  ADMUX = 0;                      //reset multiplexer settings
  ADCSRA = (1 << ADEN);             //reset ADC Control (ADC Enable 1, everything else 0)
  ADMUX |= (0 << REFS2) | (0 << REFS1) | (0 << REFS0);  //Vcc as voltage reference --not necessary, but a reminder
  ADMUX |= (1 << ADLAR);                    //8-Bit ADC in ADCH Register
  ADMUX |= (1 << MUX1) | (1 << MUX0);       //select ADC3 (control input)
  ADCSRA |= (1 << ADSC);                    // start conversion

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
  TCCR0B = (1 << CS00);                // no prescale
  TIMSK = (1 << TOIE0);                // Enable Interrupt on overflow

  //Pin interrupt Generation
  GIMSK |= (1 << PCIE);                // Enable Pin Change Interrupt
  PCMSK |= (1 << PCINT1);              // on pin 1

  sei();                               // Interrupts ON (enable interrupts globally)
  
  //go for it!
  initializeAlgoParameters(currentAlgo);
  digitalWrite(0, HIGH);               // turn LED ON

}

ISR(TIMER0_OVF_vect) {                 // Timer 0 interruption - changes the width of timer 1's pulse to generate waves 
  
  algo(currentAlgo);
}

ISR(PCINT0_vect) {                     // PIN Interruption - has priority over COMPA; this ensures that the switch will work
  TIMSK = (0 << TOIE0);                // Disable sound generation Interrupt 
  inputButtonValue = PINB & 0x02;      // Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

void loop() {
  OCR1B = sample;
  checkButton();
  setParameter();
}

void algo(int currentAlgo) { // 0 - 3: Algorithms by enveloop, based on previous work by Viznut, Tejeez & al (see sources below)
  switch (currentAlgo) {
    case 0:
      sample = t*(t>>parameters[0]) ^ (t<<parameters[1] ^ t); // 14-8-1
      t = t + parameters[2];
      break;
    case 1:
      sample = t+(t>>parameters[0]) ^ (t<<parameters[1] & t); // 1-10-1
      t = t + parameters[2];
      break;
    case 2:
      sample += ((((t<<(t>>parameters[0]))&t)>>7|t)&parameters[1]); // 
      t = t + parameters[2];
      break;
    case 3:
      sample = t&(t>>parameters[0])>>parameters[1]; // 
      t = t + parameters[2];
      break;
    case 4:                           // xorshift noise
      t ^= (t << parameters[0]);
      t ^= (t >> parameters[1]);
      sample ^= (t << parameters[2]); // 2-5-3
      if (t == 0) t = 1;              // avoids "extinguishing the noise"
      break;
  }
}

void initializeAlgoParameters(int currentAlgo) {
  t = 1;                             // for case 4, noise, with needs it initialized to 1
  currentParameter = 0;
  switch (currentAlgo) {
    case 0:
      parameters[0] = 14;
      parameters[1] = 8;
      parameters[2] = 1;
      break;
    case 1:
      parameters[0] = 10;  //1 10 1
      parameters[1] = 2;
      parameters[2] = 1;
      break;
    case 2:
      parameters[0] = 7;
      parameters[1] = 9;
      parameters[2] = 2;
      break;
    case 3:
      parameters[0] = 2;
      parameters[1] = 8;
      parameters[2] = 1;
      break;
    case 4:
      parameters[0] = 2;
      parameters[1] = 5;
      parameters[2] = 3;
      break;
  }
}


void setParameter() {
  controlInput = ADCH >> 4; //0 to 15
  if (parameterChange == true) parameters[currentParameter] = controlInput;  
  
  else if (controlInput == parameters[currentParameter]) {  //check control input against stored value. If the value is the same (because we have moved the knob to the last known position for that parameter),
      parameterChange = true;                               //it is ok to change the value :)
  }
  
  ADCSRA |=  (1<<ADSC);                                     //start next conversion
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
          if (currentParameter == 3) currentParameter = 0; //3 parameters, indexes from 0 to 2
          flashLEDSlow(currentParameter + 1);             //parameter 0 - flash once, etc                     
        }   
        else {                         // more than one click
          currentAlgo++;
          if (currentAlgo == 5) currentAlgo = 0; 
          initializeAlgoParameters(currentAlgo);
          flashLEDSlow(1); 
          additionalClicks = 0;        
        }
        button_delay = 0;
        button_delay_b = 0;
        hold = false;
        parameterChange = false;  //reset parameter change condition so that after we press the button, the newly selected parameter won't immediately change (see setParameter)
        TIMSK = (1 << TOIE0);     // Enable Interrupt
      }
    }
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

void flashLEDSlow(int times) {
  for (int i = 0; i < times; i++){
    _delay_ms(100);
    digitalWrite(0, LOW);
    _delay_ms(100);
    digitalWrite(0, HIGH);
  }
}


/*Algorithm Sources for reference 

https://gist.github.com/mvasilkov/352f863e38989aa89127

http://www.windows93.net/    (within the Byte Beat program)

Try them here: http://wurstcaptures.untergrund.net/music/ , or here:

https://greggman.com/downloads/examples/html5bytebeat/html5bytebeat.html ) 


*/


