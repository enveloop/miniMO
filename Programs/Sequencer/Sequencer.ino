/*
//************************
//*   miniMO SEQUENCER   *
//*   2016 by enveloop   *
//************************
//
   http://www.envelooponline.com/minimo
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

I/O
  Outputs: control voltage for frequency
  Input 1: frequency (during calibration)
  Input 2 (used as output): gate (note ON/OFF)

MODES OF OPERATION
  PLAY (default)
  The sequencer plays through all the steps
    -Knob: change tempo
       -miniMO waits until you reach the value it has currently stored
    -Single click: go to EDIT mode
      -To edit a step, click the button during the PREVIOUS step
    -Triple click: go to frequency calibration mode (see below)
    -The LED turns ON and OFF to mark the steps
  Each time you turn the module OFF and ON, miniMO plays a new random sequence
  
  EDIT 
  The sequencer repeats the current step indefinitely at the tempo set in PLAY mode
    -Knob: change note frequency  
      -miniMO waits until you reach the value it has currently stored
    -Single click: go to PLAY mode 
    -Double click: change note length
      -Three note lengths are possible: half(staccato), silence, and full(legato), in this order.
      -By default, the notes are set to half.
      -Full notes don't send a "note off" signal. This is most useful in combination with the envelope module.
    -The LED stays ON continuously 
   
  FREQUENCY CALIBRATION (With an OSC Module)
  When you enter this mode, miniMO starts an automatic procedure and calibrates its output to send values that result in the target frequencies defined in the program
    -Before calibration,
      -Connect any output to the input 1 in the OSC module
      -Connect the input 1 to any output in the OSC module
      -Connect the input 2 to the input 2 in the OSC module
      -Move the SEQ's knob halfway (pointing at input 2)
    -Click the SEQ module's button two or three times to start the calibration procedure in the sequencer 
    -Start the OSC calibration procedure  
      -A series of high and low beeps are heard; this calibrates the OSC module
      -A rising pitch is heard; this calibrates the SEQ module
    -When the pitch stops rising, calibration is finished. A sequence starts playing 
      -Disconnect the cable int input 1 in the SEQ module
  miniMO automatically saves the calibrated values to memory and recalls them if you turn it OFF and ON again 

  CALIBRATION TROUBLESHOOTING
  Problem: Can't start calibration in the OSC module
    -Solution: Make sure to start SEQ calibration before OSC calibration
  Problem: Calibration gets stuck in an endless loop from the lowest note up
    -Solution: Recheck cable connections and knob positions, then repeat calibration
  Problem: Calibration gets stuck in an endless loop towards the highest pitches
    -Solution: Move the knob in the controller clockwise by a small amount and repeat calibration
  Problem: After calibration, moving the knob in the controller gives strange sounds
    -Solution: Disconnect the cable between the OSC output and the SEQ input
  Problem: Calibration is lost after turning the OSC OFF and ON again
    -Solution: Disconnect the cable connected to the OSC's input 2 before turning it OFF
               Alternatively, turn OFF the SEQ before the OSC,
               or, make sure that the SEQ is not sending a note before turning OFF the OSC

  BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
*/

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

bool noteChange = false;
bool tempoChange = false;

volatile unsigned int globalTicks = 0;

//button interrupt
volatile bool inputButtonValue;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
int additionalClicks = 0;      //variable to see how many times we click after the first

//calibration data
/*
const int PROGMEM arrayLength = 37; 
const int PROGMEM targetFrequencies[arrayLength] = {  //int array, so we read it with pgm_read_word_near(targetFrequencies + j) later on
  110,  117,  123, 131,  139,  147,  156,  165,  175,   
  185,  196,  208, 220,  233,  247,  262,  277,  294,       
  311,  330,  349, 370,  392,  415,  440,  466,  494,         
  523,  554,  587, 622,  659,  698,  740,  784,  831,  880 //chromatic A2 to A5
};
*/
const int PROGMEM arrayLength = 13; 
const int PROGMEM targetFrequencies[arrayLength] = {  //int array, so we read it with pgm_read_word_near(targetFrequencies + j) later on
  110, 147, 165, 196,  
  220, 294, 330, 392,      
  440, 587, 659, 784, 880 //tetratonic A2 to A5
};

int calibratedFrequencies[arrayLength];

bool calibrating = false;
volatile unsigned long Count;
bool found = false;
bool tested = false;

//sequencer data
const int PROGMEM maxSteps = 4;             //this parameter sets the number of steps in the sequence
const int PROGMEM stepParams = 2;           //parameters: note-length
bool play = true;

const int totalStepInfos = (maxSteps * stepParams); //all the parameters in all the steps

int stepInfo[totalStepInfos]; //array to hold all those parameters

int currentStep = 0;

int tempo = 120;   //bpm
const int PROGMEM minTempo = 60;
unsigned int stepDelay = 7500/tempo;

void setup() {
  //disable USI to save power as we are not using it
  PRR = 1<<PRUSI;
  
  //calibrate the internal oscillator
  OSCCAL = 161;
  
  //set LED pin and check the battery level
  pinMode(0, OUTPUT); //LED
  checkVoltage();
  ADMUX = 0;                      //reset multiplexer settings
  
  //set pins
  pinMode(3, INPUT);  //analog input (potentiometer plus external input 1)
  pinMode(1, INPUT);  //digital input (push button)
  pinMode(4, OUTPUT); //Note
  pinMode(2, OUTPUT); //Marked as IN2 in the PCB- Gate
 
  //disable digital input in pins that do analog conversion
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3
 
  //PWM Generation -timer 1
  GTCCR  = (1<<PWM1B)|(1<<COM1B1);    //PWM, output on pb1, compare with OCR1B, reset on match with OCR1C
  OCR1C  = 0xff;
  TCCR1  = (1<<CS10);                 // no prescale

  //Pin interrupt Generation
  GIMSK |= (1<<PCIE);                  // Enable Pin Change Interrupt 
  PCMSK |= (1<<PCINT1);                // on pin 01 
  
  //Timer Interrupt Generation -timer 0                                                          
  TCCR0A = (1<<WGM01);                 //Clear Timer on Compare (CTC) with OCR0A
  TCCR0B = (1<<CS02) ;                 // prescale by 256
  OCR0A = 250;                         //125hz https://www.easycalculation.com/engineering/electrical/avr-timer-calculator.php
  TIMSK = (1 << OCIE0A);               // Enable Interrupt on compare with OCR0A
  
  memoryToArray(calibratedFrequencies, arrayLength); //reads the last calibration values from memory and places them in the calibration array
  
  initSteps(500);                      //initialize step information. 500 is the memory address for the random Seed 
  
  sei();
     
  digitalWrite(0, HIGH);      //let there be light
}

//PIN Interruption
ISR(PCINT0_vect) {                        
   inputButtonValue = digitalRead(1);
   Count++;
}

//Timer interruption
ISR(TIMER0_COMPA_vect) {
  globalTicks++;
}

void loop() {
 if (!calibrating)
   sendSequence();
}

void sendSequence(){
  if (play){
    stepDelay = 7500/tempo;         //we set the new tempo during the step, but we only apply the setting after the step -otherwise the division grinds everything to a halt
    sendStep(currentStep); 
    currentStep++;
    if (currentStep >= (maxSteps)) currentStep = 0;
  }
  else if(!play) {  
    //setNoteFreq(3); 
    //checkButton();
    sendStepInPause(currentStep);
  }
}

//sends the selected step continuously for editing
void sendStepInPause(int currentStep){
  globalTicks = 0;
  int currentStepNote = stepInfo[(currentStep * stepParams)];
  int currentStepLength = stepInfo[(currentStep * stepParams) + 1];
  int this_delay = stepDelay;
  unsigned int this_step; 
  
  digitalWrite(0, HIGH);          //turn LED on
  
  if (currentStepLength == 0){
    digitalWrite(2, LOW);    //off
    while ((globalTicks - this_step) < stepDelay){
      checkButton();
    } 
  }
  
  if (currentStepLength == 255){  //full note
    digitalWrite(2, HIGH);    //send note
    while ((globalTicks - this_step) < stepDelay){
      checkButton();
      setNoteFreq(3);                                           //only set frequency here (otherwise double click won't work)
      OCR1B = currentStepNote;                                  //send note
    } 
  }
  
  else if (currentStepLength == 127) {    //half note
  
    this_delay = stepDelay >> 1;    //full or half note ( >> is /2^1)
    
    this_step = globalTicks;
    digitalWrite(2, HIGH);                                      
    while ((globalTicks - this_step) < this_delay){
      checkButton(); 
      setNoteFreq(3);                                           //only set frequency here (otherwise double click won't work)
      OCR1B = currentStepNote;                                  //send note
    }
    this_step = globalTicks;
    digitalWrite(2, LOW);
    while ((globalTicks - this_step) < (stepDelay - this_delay)){
      checkButton();
    }
  }
}

//sends the current step
void sendStep(int currentStep){
  globalTicks = 0;
  int ticksToLEDOff = 5;                                            //ticks after which the LED switches OFF
  int currentStepNote = stepInfo[(currentStep * stepParams)];
  int currentStepLength = stepInfo[(currentStep * stepParams) + 1];
  int this_delay = stepDelay;
  unsigned int this_step; 
  
  digitalWrite(0, HIGH);          //turn LED on
  
  if (currentStepLength == 0){     //silence
    digitalWrite(2, LOW);    //off
    while ((globalTicks - this_step) < stepDelay){
      checkButton();
      setTempo(3);
      if (globalTicks == ticksToLEDOff) digitalWrite(0, LOW);
    } 
  }
  else if (currentStepLength == 255){  //full note (no note off)
    digitalWrite(2, HIGH);              //send note
    while ((globalTicks - this_step) < stepDelay){
      checkButton();
      setTempo(3);
      OCR1B = currentStepNote;
      if (globalTicks == ticksToLEDOff) digitalWrite(0, LOW);
    } 
  }
  else if (currentStepLength == 127) {
  
    this_delay = stepDelay >> 1;    //half note
    
    this_step = globalTicks;
    digitalWrite(2, HIGH);                                      
    while ((globalTicks - this_step) < this_delay){
      checkButton(); 
      setTempo(3);                                
      OCR1B = currentStepNote;                                  //send note
      if (globalTicks == ticksToLEDOff) digitalWrite(0, LOW);
    }
    this_step = globalTicks;
    digitalWrite(2, LOW);                                        //note off after half a step
    while ((globalTicks - this_step) < (stepDelay - this_delay)){
      checkButton();
      setTempo(3);
      if (globalTicks == ticksToLEDOff) digitalWrite(0, LOW);
    }
  }
}

void setNoteLength() {
  int currentStepLength = stepInfo[(currentStep * stepParams) + 1];
  switch (currentStepLength) {
    case 0:
      digitalWrite(0, HIGH);
      stepInfo[(currentStep * stepParams) + 1] = 255;
      break;
    case 127:
      stepInfo[(currentStep * stepParams) + 1] = 0;
      break;
    case 255:
      stepInfo[(currentStep * stepParams) + 1] = 127;
      break;  
  } 
}

void setTempo(int pin) {
  noteChange = false;
  int tempoRead = analogRead(pin) + minTempo;    
  if(tempoChange == false){
    if(tempoRead == tempo){
      tempoChange = true;
    }
  }
  else if(tempoChange ==true){
    tempo = tempoRead;
  }
}

/*
void setNoteFreq(int pin) {  //doesn't wait to get to the previous value
    digitalWrite(2, HIGH);
    tempoChange = false;
    int noteRead = analogRead(pin)>>2;   //0-255
    stepInfo[(currentStep * stepParams)] = noteMap(noteRead);
    OCR1B = noteMap(noteRead);
}*/

void setNoteFreq(int pin) {
  digitalWrite(2, HIGH);
  tempoChange = false;
  int noteRef = stepInfo[(currentStep * stepParams)];
  int noteRead = analogRead(pin)>>2;
  if (noteChange == false){
    OCR1B = noteRef;
    if (noteRead == noteRef){
      noteChange = true;
    }
  }
  else if (noteChange == true){
    stepInfo[(currentStep * stepParams)] = noteMap(noteRead); 
    OCR1B = noteMap(noteRead);    
    }     
}

int noteMap(int note){ //returns the closest calibrated value
  int result;
  for (int i = 0; i < arrayLength; i++)
  {
    if (abs(note - calibratedFrequencies[i]) < abs(note - result))
    result = calibratedFrequencies[i];
   }
  return result;
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
              
              if (play) play = false; 
              else play = true;

              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
          }
          else if (additionalClicks == 1) {             //button pressed again (double click),         
            
  	    if (!play) { 
               setNoteLength();  
            }
            
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
            }
          else if (additionalClicks > 1 ) {                 //button pressed at least twice more(triple click or more)
            
            if (play) calibrate();                          //only when playing to avoid triggering it by mistake when changing parameters
            
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

void calibrate() {
  TCCR0B = (1 << CS01) | (1 << CS00);             //prescale by 64
  OCR0A = 125;                                    //1000hz - 1000 ticks per second
  sendCalibration();                              //sends the max and min values (this is to calibrate the oscillator)
  PCMSK = (1 << PCINT3);                          //interrupt in input 3 (frequency)
  calibrating = true;
  int nextI = 0;
  for (int j = 0; j < arrayLength; j++) {                 //for every target frequency in the array
    for (int i = nextI; i < 256; i++) {          //tests voltages, starting with the last voltage that gave a valid frequency
      found = false;
      tested = false;
      OCR1B = i;                                //sends the voltage
      while (tested == false) {
        testFrequency(pgm_read_word_near(targetFrequencies + j)); 
      }
      if (found) {
        calibratedFrequencies[j] = i;    //puts the calibration value for this note in the array
        nextI = i;
        break;
      }
    }
  }
  //once all the notes are calibrated
  arrayToMemory(calibratedFrequencies, arrayLength);  //save the calibration array to memory   
  TCCR0B = (1<<CS02) ;                       // prescale by 256
  OCR0A = 250;                               //125hz -back to the regular setings
  PCMSK = (1 << PCINT1);                     //interrupt back to button
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
  //digitalWrite(0, LOW);
  digitalWrite(2, HIGH);
  int i = 0;
  for (int  j = 0; j < 5; ++j){ //alternates 5 times between max and min
    i = 0;
    for (i = 0; i < 256; ++i) {
      OCR1B = 255; 
      _delay_ms(2);
    }
    i = 0;
    for (i = 0; i < 256; ++i) {
      OCR1B = 0; 
      _delay_ms(2);
     }
  }
}

void initSteps(int address){  //initialize steps' info to random notes
  unsigned int rSeed = eeprom_read_word((uint16_t*)500);
  rSeed++;
  srand(rSeed);

  for (int i = 0; i < maxSteps; i++) {
    int randomValue = rand() >> 7;                            //rand (max value 32767) >> 7 to give max 255
    if (randomValue < 10) randomValue = 10;                   //avoiding low values (solves a bug whereby sometimes a very high note was produced)
    int note = noteMap(randomValue);                          //find the closest calibrated value to the random number 
    
    stepInfo[i * stepParams] =  note;                         
    stepInfo[(i * stepParams) + 1 ] = 127;                    //all half notes
  }
  eeprom_update_word((uint16_t*)500, rSeed);
}
 
void checkVoltage(){ //voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5 
  
  ADMUX |= (1<<ADLAR);                  //Left adjust result (8 bit conversion stored in ADCH)
  ADMUX |= (1<<MUX3)|(1<<MUX2);         //1.1v input
  delay(250);                           // Wait for Vref to settle
  ADCSRA |= (1<<ADSC);                  // Start conversion
  while (bit_is_set(ADCSRA,ADSC));      // wait while measuring
  if (ADCH > 103)                       //aprox 2.6
    flashLED(8,100);
  else 
    flashLED(1,250);
}

void flashLED (int times, int gap) {     //for voltage check (uses regular delay)
  for (int i=0; i<times; i++)
  {
    digitalWrite(0, HIGH);                 
    delay(gap);
    digitalWrite(0, LOW);
    delay(gap);
  }
} 
