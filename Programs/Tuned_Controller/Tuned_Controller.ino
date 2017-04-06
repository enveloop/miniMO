/*
//*******************************
//*    miniMO Tuned Controller  *
//*     2016 by enveloop        *
//*******************************
//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license:
   http://creativecommons.org/licenses/by/4.0/
//

I/O
  1&2: Outputs - control voltage for frequency
  3: Input - frequency (during calibration) 
  4: Output - gate (note ON/OFF) 
 
MODES OF OPERATION
  PLAY (default)
    -Knob: change frequency
    -Single click: toggle between note ON and OFF
      -If the note is ON, the LED is also ON (and the other way round)
    -Double click: frequency calibration with OSC module(see below)
    -Triple click: frequency calibration with OSC module(see below)
 
  FREQUENCY CALIBRATION (With an Oscillator Module)
  When you enter this mode, miniMO starts an automatic procedure and calibrates its output to send values that result in the target frequencies defined in the program
    -Before calibration,
      -Connect the controller's I/O 1 or 2 to the oscillator's I/O 3 (controller note output to oscillator frequency input)
      -Connect the controller's I/O 3 to the oscillator's I/O 1 or 2 (controller calibration input to oscillator frequency output)
      -Connect the controller's I/O 4 to the oscillator's I/O 4      (controller gate output to oscillator volume output)
      -Turn the controller's knob all the way counter-clockwise and make sure the arrow points at the button 
      -Turn the controller's knob halfway (arrow pointing at I/O 4)
    -Click the controller module's button two or three times to start the calibration procedure in the controller 
    -Start the oscillator calibration procedure (refer to oscillator manual on how to set it up) 
      -A series of high and low beeps are heard; this calibrates the oscillator module
      -A rising pitch is heard; this calibrates the controller module
    -When the pitch stops rising, calibration is finished 
      -Disconnect the cable from I/O 3 in the controller module
      -Turn the controller OFF and ON again. A sequence starts playing ^_^
      
  miniMO automatically saves the calibrated values to memory and recalls them if you turn it OFF and ON again 
   
  
  CALIBRATION TROUBLESHOOTING
  Problem: Calibration gets stuck in an endless loop from the lowest note up
    -Solution: Recheck cable connections and knob positions, then repeat calibration
  Problem: Calibration gets stuck in an endless loop towards the highest pitches
    -Solution: Move the knob in the controller clockwise by a small amount and repeat calibration
  Problem: After calibration, moving the knob in the controller gives strange sounds
    -Solution: Disconnect the cable between the oscillator output and the controller input
  Problem: Calibration is lost after turning the oscillator OFF and ON again
    -Solution: Disconnect the cable connected to the oscillator's input 2 before turning it OFF
               Alternatively, turn OFF the Controller before the oscillator,
               or, make sure that the controller is not sending a note before turning OFF the oscillator
                   
  BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
*/

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

volatile unsigned int globalTicks = 0;
volatile unsigned long Count;
bool found = false;
bool tested = false;

//button interrupt
volatile bool inputButtonValue;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
int additionalClicks = 0;      //variable to see how many times we click after the first

//calibration data

const int PROGMEM arrayLength = 37;
const int PROGMEM targetFrequencies[arrayLength] = {  //int array, so we read it with pgm_read_word_near(targetFrequencies + j) later on
  110,  117,  123, 131,  139,  147,  156,  165,  175,   
  185,  196,  208, 220,  233,  247,  262,  277,  294,       
  311,  330,  349, 370,  392,  415,  440,  466,  494,         
  523,  554,  587, 622,  659,  698,  740,  784,  831,  880 //A2 to A5
};
/*
const int PROGMEM arrayLength = 13; 
const int PROGMEM targetFrequencies[arrayLength] = {  
  110, 147, 165, 196,  
  220, 294, 330, 392,      
  440, 587, 659, 784, 880 //tetratonic A2 to A5
};
*/

int calibratedFrequencies[arrayLength];
bool calibrating = false;

void setup() {
  
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion
  
  //osc calibration 
  OSCCAL = 161;
  
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //Note pitch
  pinMode(2, OUTPUT); //Note trigger
  pinMode(3, INPUT);  //Analog- freq Input
  pinMode(1, INPUT);  //Digital input (push button)
  
  checkVoltage();
  ADMUX = 0;                             //reset multiplexer settings
  
  cli();                                 // Interrupts OFF (disable interrupts globally)

  GTCCR  = (1 << PWM1B) | (1 << COM1B1); // PWM, output on pb1, compare with OCR1B, reset on match with OCR1C
  OCR1C  = 0xff;                         // 255
  TCCR1  = (1 << CS10);                  // no prescale

  GIMSK = (1 << PCIE);    // Enable Pin Change Interrupt
  PCMSK = (1 << PCINT1);  // on pin 1


  //Timer Interrupt Generation -timer 0
  TCCR0A = (1 << WGM01);               //Clear Timer on Compare (CTC) with OCR0A
  TCCR0B = (1 << CS01) | (1 << CS00);  // prescaled by 64
  OCR0A = 0x7d;                         //0x7d = 125 //1000hz - 1000 ticks per second https://www.easycalculation.com/engineering/electrical/avr-timer-calculator.php
  TIMSK = (1 << OCIE0A);               // Enable Interrupt on compare with OCR0A
  
  memoryToArray(calibratedFrequencies, arrayLength); //reads the last calibration values from memory and places them in the calibration array
  
  sei();                               // Interrupts ON (enable interrupts globally)
  
  //go for it!
  digitalWrite(0, HIGH); // turn LED ON
  digitalWrite(2, HIGH); // turn note ON
  
  
}

ISR(PCINT0_vect) {                 //PIN Interruption - has priority over Timer 0; this ensures that the switch will work   
  inputButtonValue = PINB & 0x02;  //Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02)
  Count++;                         //This interruption happens both on rising and falling. When calibrating, we use the analog input as source for the interrupt, and freq = interruptions (Count) in a second/2 
}

ISR(TIMER0_COMPA_vect) {           //1000 ticks per second
  globalTicks++;
}

void loop ()   {
  setNote(3);
  checkButton();
}

void setNote(int pin) {
  if (!calibrating){
    int noteRead = analogRead(pin) >> 2;   //0-255
    OCR1B = noteMap(noteRead);
  }
}

void toggleNote(){
  if (digitalRead(2) == HIGH){
    digitalWrite(2, LOW);
    digitalWrite(0, LOW);
  } 
  else {
    digitalWrite(2, HIGH);
    digitalWrite(0, HIGH);
  }
}

int noteMap(int note){
  int result;
  for (int i = 0; i < arrayLength; i++)
  {
    if (abs(note - calibratedFrequencies[i]) < abs(note - result))
    result = calibratedFrequencies[i];
   }
  return result;
}

void calibrate() {
  sendCalibration();                              //sends the max and min values (this is to calibrate the oscillator)
  PCMSK = (1 << PCINT3);                          //source for PIN interruption: input 3 (frequency)
  calibrating = true;
  int nextI = 0;
  for (int j = 0; j < arrayLength; j++) {        //for every target frequency in the array
    for (int i = nextI; i < 256; i++) {          //tests voltages, starting with the last voltage that gave a valid frequency
      found = false;
      tested = false;
      OCR1B = i;                                //sends the voltage
      while (tested == false) {
        testFrequency(pgm_read_word_near(targetFrequencies + j)); 
      }
      if (found) {
        calibratedFrequencies[j] = i;            //puts the calibration value for this note in the array
        nextI = i;
        break;
      }
    }
  }
  //once all the notes are calibrated
  arrayToMemory(calibratedFrequencies, arrayLength);  //save the calibration array to memory   
  PCMSK = (1 << PCINT1);                              //interrupt source back to button
  calibrating = false;
}

void arrayToMemory(int array[], int arraySize){
  int j = 1; 
  int offset = 2; //two bytes for each int
  for (int i = 0; i < arraySize; i++) {
    eeprom_update_word((uint16_t*)j, array[i]);
    j = j + offset;
  }
}

void memoryToArray(int array[], int arraySize){
  int j = 1;
  int offset = 2; //two bytes for each int
  for (int i = 0; i < arraySize; i++) {
    array[i] = eeprom_read_word((uint16_t*)j);
    j = j + offset;
  } 
}

void testFrequency(int target) { //in 1 second, count = freq * 2
  globalTicks = 0;
  Count = 0;
  while (globalTicks < 125);  //eigth of a second
  if (Count > (target >> 2)) found = true;
  tested = true;
}

void sendCalibration() {    //sends the max and min values (this is to calibrate the oscillator)
  digitalWrite(2, HIGH);
  int i = 0;
  for (int  j = 0; j < 5; ++j){ //alternates 5 times between max and min
    i = 0;
    for (i = 0; i < 256; ++i) {
      OCR1B = 0xff;               //255
      _delay_ms(2);
    }
    i = 0;
    for (i = 0; i < 256; ++i) {
      OCR1B = 0; 
      _delay_ms(2);
     }
  }
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

              toggleNote();

              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
          }
          else if (additionalClicks == 1) {             //button pressed again (double click),         
            
            calibrate();
            
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
            }
          else if (additionalClicks > 1 ) {                 //button pressed at least twice (triple click or more)
            
            calibrate();
            
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
  ADMUX = (0<<REFS1)|(0<<REFS0);        //Vcc as reference
  ADMUX |= (1<<ADLAR);                  //Left adjust result (8 bit conversion stored in ADCH)
  ADMUX |= (1<<MUX3)|(1<<MUX2);         //1.1v input
  delay(250);                           // Wait for Vref to settle
  ADCSRA |= (1<<ADSC);                  // Start conversion
  while (bit_is_set(ADCSRA,ADSC));      // wait while measuring
  if (ADCH > 103)                       //approx 2.6
    flashLED(8,100);
  else 
    flashLED(1,250);
}

void flashLED (int times, int gap) {     //for voltage check only (uses regular delay)
  //delay(250);
  for (int i=0; i<times; i++)
  {
    digitalWrite(0, HIGH);                 
    delay(gap);
    digitalWrite(0, LOW);
    delay(gap);
  }
} 
