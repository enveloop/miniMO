/*
//*****************************
//*   miniMO MIDI SEQUENCER   *
//*     2017-18 by enveloop   *
//*****************************
//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

MIDI OUT CABLE: https://www.midi.org/specifications/item/midi-din-electrical-specification

Resistors: 10Ω(data, pin5), 33Ω(power, pin4)

//

I/O
  2: Output - MIDI notes
  4: Output (master) / Input (slave) - gate trigger

TRIGGER MODES
  MASTER (default)
  The sequencer sends a trigger every step through I/O 4, also during editing
  
  SLAVE
  The sequencer advances a step when it detects a trigger through I/O 4 (tested 3 and 5V)
    -To set the sequencer to Slave Mode, turn the module ON while pressing the button, and release the button after the battery check 
       -The LED turns OFF -now the module is waiting for a trigger
    -All regular functions are available in Slave Mode
       -Tempo Control is also available 
          -The unit will send MIDI notes at the internal tempo, if it's slowest than the external 
          -The internal tempo also determines the note length (very fast internal tempos produce shorter notes)    

OPERATION MODES
  PLAY (default)
  The sequencer plays through all the steps. The LED turns ON and OFF to mark each step
    -Knob: change tempo / transpose pattern
       -When you hop between parameters, miniMO waits until you reach the last value it has currently stored to start effecting changes
    -Single click: go to EDIT mode
      -To edit a step, click the button during the PREVIOUS step
      -The pattern freezes in the step after you click, which becomes ready to edit
    -Double click: change play mode between regular and reverse
    -Triple click: change the number of octaves the pattern is played across 
    -Click and Hold (hold for about 1s): change knob's function between tempo / transposition control 
    
  Each time you turn the module OFF and ON, miniMO plays a new random sequence
  
  EDIT 
  The sequencer repeats the current step indefinitely at the tempo set in PLAY mode. The LED stays ON continuously
    -Knob: change note  
      -miniMO waits until you reach the note it has currently stored to start effecting changes
    -Single click: go to PLAY mode 
    -Double click: change note length between half (staccato), full (legato), and silence, in this order
      -By default, the first note is set to full (to mark the beginning of the pattern), and the other notes are set to half
   
  BATTERY CHECK
  When you switch the module ON,
    -If the LED blinks once, the battery is OK
    -If the LED blinks fast several times, the battery is running low
    
  NOTES&TROUBLESHOOTING
  This program requires a modified version of SoftwareSerial to run (see below)
  The module's "wait until the knob reaches the last stored value" behavior might be a bit unresponsive at low tempos
  In Slave mode, the maximum rate is not as high as in Master Mode  
*/

#include <SoftwareSerialminiMO.h>  //get this library at https://github.com/enveloop/miniMO/tree/master/Libraries
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

SoftwareSerial midiSerial(-1, 4); // digital pins for soft serial RX & TX. (-1 = OFF)

//button interrupt
volatile bool inputButtonValue;

//timer interrupt
volatile unsigned int globalTicks = 0;

//button press control
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
byte additionalClicks = 0;      //variable to see how many times we click after the first

//parameter change control
bool noteChange = false;
bool tempoChange = false;
bool transposeChange = false;

//sequencer data - USER EDITING ENCOURAGED
//////////////////////////////////////////
const byte PROGMEM midiChannel = 1;                           //midi channel
const byte PROGMEM noteVelocity = 100;                        //default note velocity
const byte PROGMEM maxSteps = 6;                              //number of steps in the sequence

const int PROGMEM targetNotesArrayLength = 9;                 //number of items in the target notes array
const int PROGMEM targetNotes[targetNotesArrayLength] =       //we store them in program memory to avoid using too much dynamic memory if the array is long
{  
  52,55,59,60,64,67,71,72,76,                                 //THESE ARE THE MIDI NOTES WE'LL USE IN THE SEQUENCES
};

const byte maxOctaves = 3;                                    //values: 0 (no change), 1 octave, 2 octaves               
const byte transposeValues[8] = {-7, -5, -2, 0, 2, 5, 7, 12}; //transposition intervals, in semitones
///////////////////////////////////////////

//other sequencer data
int tempo = 120;   //bpm
const int PROGMEM minTempo = 60;
unsigned int stepDelay = 7500/tempo;

const byte PROGMEM stepParams = 2;                  //parameters: note-length
const int totalStepInfos = (maxSteps * stepParams); //all the parameters in all the steps
int stepInfo[totalStepInfos];                       //array to hold all those parameters

bool play = true;
bool transposeMode = false;
bool playReverse = 0;  

byte playOctave = 0;  
byte limitOctave = 0;  

const byte transposeModifierBaseDefault = 3; 
byte transposeModifierBase = transposeModifierBaseDefault;  //reading from potentiometer. By default is 3, the 4th value in transposeValues[], which gives no transposition.
byte transposeModifier = 0;                                 //transposition interval derived from applying the pot reading to a table of intervals

int currentStep = 0;

bool slave = 0;
bool readyToRetrigger = 0;
int val = 0;

void setup() 
{  
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion
 
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //Note output
  pinMode(2, OUTPUT); //Note output
  pinMode(3, INPUT);  //Speed input/ Note input during editing
  pinMode(1, INPUT);  //Digital input (push button)
  
  checkVoltage();
  ADMUX = 0;                           //reset multiplexer settings
  
  cli();                               // Interrupts OFF (disable interrupts globally)
  
  //Pin interrupt Generation
  GIMSK |= (1<<PCIE);                  // Enable Pin Change Interrupt 
  PCMSK |= (1<<PCINT1);                // on pin 01 
  
  //Timer Interrupt Generation -timer 0                                                          
  TCCR0A = (1<<WGM01);                 //Clear Timer on Compare (CTC) with OCR0A
  TCCR0B = (1<<CS02) ;                 // prescale by 256
  OCR0A = 0xfa;                        //0xfa = 250 //125hz https://www.easycalculation.com/engineering/electrical/avr-timer-calculator.php
  TIMSK = (1 << OCIE0A);               // Enable Interrupt on compare with OCR0A
  
  initSteps(500);                      //initialize step information -500 is the memory address for the random seed
  
  sei();

  //Start as MIDI master or Slave
  if (digitalRead(1) == HIGH) {                            //If we are pressing the button
    digitalWrite(0, HIGH);                                 //LED ON
    _delay_ms(2000);                                      
    digitalWrite(0, LOW);
    if (inputButtonValue == 0) {
      slave = 1;                //If we released the button, set Slave mode
      pinMode(2, INPUT);        //Gate input (slave)
    }
  }
  else {                                                   
    slave = 0;                  //master mode
    pinMode(2, OUTPUT);         //Gate output (master)
  }

  midiSerial.begin(31250);
}


ISR(PCINT0_vect) {                  //PIN Interruption - has priority over Timer 0; this ensures that the switch will work   
   inputButtonValue = PINB & 0x02;  //Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

ISR(TIMER0_COMPA_vect) {            //Timer0 interruption
  globalTicks++;
}

void loop() {
  sequence();
}

void sequence() {
  if (play) {
    stepDelay = 7500/tempo;                                            //we set the new tempo during the step, but we only apply the setting after the step -otherwise the division grinds everything to a halt
    if (slave) receiveStep(currentStep);
    else if (!slave) sendStep(currentStep);
  }
  else if(!play) {  
    if (slave) receiveStepInPause(currentStep);
    else if (!slave) sendStepInPause(currentStep);
  }
}

void sendStepInPause(int currentStep) {                                  //EDIT MODE - sends the selected step continuously for editing
  globalTicks = 0;
  int ticksToExtOff = 5;
  int currentStepNote = stepInfo[(currentStep * stepParams)] + (12 * playOctave) + transposeModifier;
  int currentStepLength = stepInfo[(currentStep * stepParams) + 1];
  int this_delay = stepDelay;
  unsigned int this_step; 
  
  digitalWrite(0, HIGH);                         //turn LED ON
  digitalWrite(2, HIGH);                         //send external trigger
  
  if (currentStepLength == 0)                    //silence
  { 
    digitalWrite(0, LOW);                        //turn LED OFF
    while ((globalTicks - this_step) < stepDelay)
    {
      checkButton();
      if (globalTicks == ticksToExtOff) 
      {
        digitalWrite(2, LOW); //turn ext OFF
      }
    } 
  }
  if (currentStepLength == 255) {                 //full note
    playNote(currentStepNote);
    while ((globalTicks - this_step) < stepDelay)
    {
      checkButton();
      setNoteFreq(3);                            //only set frequency here (otherwise multiple click won't work)
      if (globalTicks == ticksToExtOff) 
      {
        digitalWrite(2, LOW); //turn ext OFF
      }
    } 
    stopNote(currentStepNote);
  }
  else if (currentStepLength == 127) //half note
  {    
    this_delay = stepDelay >> 1;                 //same as stepDelay / 2^1 but more efficient
    
    this_step = globalTicks;   
    playNote(currentStepNote);    
    while ((globalTicks - this_step) < this_delay)
    {
      checkButton(); 
      setNoteFreq(3);                            //only set frequency here (otherwise multiple click won't work)
      if (globalTicks == ticksToExtOff) 
      {
        digitalWrite(2, LOW); //turn ext OFF
      }
    }
    this_step = globalTicks;
    stopNote(currentStepNote);
    while ((globalTicks - this_step) < (stepDelay - this_delay))
    {
      checkButton();
    }
  }
}

void sendStep(int currentStep) {                  //PLAY MODE - sends the current step                                 
  globalTicks = 0;
  int ticksToLEDOff = 5;                        //ticks after which the LED switches OFF
  int currentStepNote = stepInfo[(currentStep * stepParams)] + (12 * playOctave) + transposeModifier;
  int currentStepLength = stepInfo[(currentStep * stepParams) + 1];
  int this_delay = stepDelay;
  unsigned int this_step; 
  
  digitalWrite(0, HIGH);                       //turn LED ON to mark the step (even if there's a silence)
  digitalWrite(2, HIGH);                       //turn external trigger ON (follows the main LED)
  
  if (currentStepLength == 0)                  //silence
  {     
    while ((globalTicks - this_step) < stepDelay)
    {
      checkButton();
      if (!transposeMode) setTempo(3);
      else setTransposeBase(3);
      if (globalTicks == ticksToLEDOff) 
      {
        digitalWrite(0, LOW); //turn LED OFF
        digitalWrite(2, LOW); //turn ext OFF
      }
    } 
  }
  else if (currentStepLength == 255)           //full note 
  {
    playNote(currentStepNote);
    while ((globalTicks - this_step) < stepDelay)
    {
      checkButton();
      if (!transposeMode) setTempo(3);
      else setTransposeBase(3);
      if (globalTicks == ticksToLEDOff)
      {
        digitalWrite(0, LOW); //turn LED OFF
        digitalWrite(2, LOW); //turn ext OFF
      }
    }
    stopNote(currentStepNote);
  }
  else if (currentStepLength == 127)            //half note
  {
    this_delay = stepDelay >> 1;                //same as stepDelay / 2^1 but more efficient
    this_step = globalTicks;

    playNote(currentStepNote);    
    while ((globalTicks - this_step) < this_delay)
    {
      checkButton(); 
      if (!transposeMode) setTempo(3);
      else setTransposeBase(3);                           
      if (globalTicks == ticksToLEDOff)
      {
        digitalWrite(0, LOW);    //turn LED OFF
        digitalWrite(2, LOW); //turn ext OFF
      }
    }
    this_step = globalTicks;
    stopNote(currentStepNote);
    while ((globalTicks - this_step) < (stepDelay - this_delay)) 
    {
      checkButton();
      if (!transposeMode) setTempo(3);
      else setTransposeBase(3);
    }
  }
  advanceStep();
}

void receiveStep(int currentStep) {
  val = analogRead(1);
  if ((val > 700) && (readyToRetrigger == true))
  {
    readyToRetrigger = false;
    sendStep(currentStep);    
  }
  else if (val < 700) 
  {
    readyToRetrigger = true;
  }
}

void receiveStepInPause(int currentStep) {
  val = analogRead(1);
  if ((val > 700) && (readyToRetrigger == true))
  {
    readyToRetrigger = false;
    sendStepInPause(currentStep);    
  }
  else if (val < 700) 
  {
    readyToRetrigger = true;
  }
}

void advanceStep()
{
    if (playReverse == false) {
      currentStep++;
      if (currentStep == maxSteps) {                                   //when we reach the end of the sequence
        currentStep = 0;                                               //reset step number 
        transposeModifier = transposeValues[transposeModifierBase];    //apply transposition (we avoid transposing in the middle of a pattern)
        playOctave++;                                                  //add octave  
        if (playOctave > limitOctave) playOctave = 0;                   //if we reached the maximum octave, reset octave  
      }
    }
    else if (playReverse == true) {
      currentStep--;
      if (currentStep < 0) {
        currentStep = maxSteps - 1;
        transposeModifier = transposeValues[transposeModifierBase];
        playOctave++;
        if (playOctave > limitOctave) playOctave = 0;
      }
    }
}

void setTempo(int pin) {
  noteChange = false;
  transposeChange = false;
  int tempoRead = analogRead(pin) + minTempo; 
  if(tempoChange == false) {
    if(tempoRead == tempo) {
      tempoChange = true;
    }
  }
  else if(tempoChange ==true) {
    tempo = tempoRead;
  }
}

void setTransposeBase(int pin) {
  tempoChange = false;
  noteChange = false;
  int inputRead = analogRead(pin) >> 7;    //values between 0 and 7
  if (transposeChange == false) {          
    if (inputRead == transposeModifierBase) {                  
      transposeChange = true;
    }
  }
  else if (transposeChange == true) {
     transposeModifierBase = inputRead;    
  }     
}

void setNoteFreq(int pin) {
  tempoChange = false;
  transposeChange = false;
  int noteRef = stepInfo[(currentStep * stepParams)];
  int noteRead = analogRead(pin) >> 3;    //values between 0 and 127
  if (noteChange == false) {
    if (noteRead == noteRef) {
      noteChange = true;
    }
  }
  else if (noteChange == true) {
    stepInfo[(currentStep * stepParams)] = noteMap(noteRead);     
   }     
}
   
int noteMap(int reading) {//maps the readings' range to the target notes we defined 
  int chunk = 127 /  targetNotesArrayLength;
  int remappedReading = reading / chunk;                            //we get range from 0 to the array length (at max value)
  if (remappedReading == targetNotesArrayLength) remappedReading--; //prevent going out of bounds
  int note = pgm_read_word_near(targetNotes + remappedReading);     //retrieve the note in the chosen index
  
  return note;
}

void setNoteLength() {                             //reads the current value and sets it to the next: 127(half)->255(full)->0(silence)
  int currentStepLength = stepInfo[(currentStep * stepParams) + 1];
  switch (currentStepLength) {
    case 0:
      digitalWrite(0, HIGH);
      stepInfo[(currentStep * stepParams) + 1] = 127;
      break;
    case 127:
      stepInfo[(currentStep * stepParams) + 1] = 255;
      break;
    case 255:
      stepInfo[(currentStep * stepParams) + 1] = 0;
      break;  
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
        
        if (button_delay_b == 700) {
          if (additionalClicks == 0){           
            if (beenLongPressed) {                    //button released after being pressed for a while
            
              if (play) {                             //we use the long press to activate transpose mode, but not if we're editing
                if (!transposeMode)
                  transposeMode = true;              
                else {
                  transposeMode = false;
                }
              }
              
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
            else if (play) {                            
              if (!playReverse) playReverse = true;
              else playReverse = false;
            }
            
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
            }
          else if (additionalClicks > 1 ) {                 //button pressed at least twice more(triple click or more)
            
            if (play) {
              limitOctave++;
              if (limitOctave == maxOctaves) limitOctave = 0;          
            }

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

void playNote(byte note) {
    byte noteON = 0x90 + (midiChannel-1);  //0-15 
    
    midiSerial.write(noteON);
    midiSerial.write(note);
    midiSerial.write(noteVelocity);
}

void stopNote(byte note) {
    byte noteOFF = 0x80 + (midiChannel-1);  //0-15 
    
    midiSerial.write(noteOFF);
    midiSerial.write(note);
    midiSerial.write((byte)0);      //writing just 0 causes an ambiguity 
}
                
void initSteps(int address){  //initialize steps' info to random notes
  unsigned int rSeed = eeprom_read_word((uint16_t*)address);  //read the last seed we used, from memory
  rSeed++;                                                    //add one -now we have a new seed
  srand(rSeed);                                               //assign the new seed to the random generator (this way we'll have a new sequence of random values)

  for (int i = 0; i < maxSteps; i++) {
    int randomValue = (rand() >> 8) % (targetNotesArrayLength);//we get range from 0 to the array length - 1// get a random index in the array of notes we want
    int note = pgm_read_word_near(targetNotes + randomValue);  //retrieve the note in the chosen index          
    
    stepInfo[i * stepParams] =  note;                         //assign the note to the step in the sequence
    if (i == 0) stepInfo[ 1 ] = 255;        //the first note gets full length to mark the beginning of the pattern
    else stepInfo[(i * stepParams) + 1 ] = 127;               //all the other notes are all half notes
  }
  eeprom_update_word((uint16_t*)address, rSeed);              //save the seed we used
}

void checkVoltage() {                   //voltage from 255 to 0; 46 is (approx)5v, 94 is 2.8, 104-106 is 2.5
                                        //we measure a fixed value of 1.1 against Vcc, so the lower the measurement, the higher Vcc
  ADMUX = (0 << REFS1) | (0 << REFS0);  //Vcc as reference
  ADMUX |= (1 << ADLAR);                //Left adjust result (8 bit conversion stored in ADCH)
  ADMUX |= (1 << MUX3) | (1 << MUX2);   //1.1v input
  delay(250);                           // Wait for Vref to settle
  ADCSRA |= (1 << ADSC);                // Start conversion
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
