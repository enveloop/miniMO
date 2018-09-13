/*
/**********************************
/*    miniMO Chiptune Player      *
/*        2018 by enveloop        *
/**********************************
//
            Adapted From
            Tiny Synth v2
        by David Johnson-Davies 
   http://www.technoblogy.com/show?Q7H
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/

Controls:
Single Click:  play next tune. IMPORTANT: wait until the LED turns off to release button. Resets tempo modifications and repeat settings
Double click:  rewind current tune. (You only need to wait until the LED turns off for the first click). 
Triple Click:  rewind the whole playlist and stop.
Click and Hold )until the LED lights up again): turn autorepeat ON or OFF for the current tune. Resets tempo modifications and repeat settings
Potentiometer: change tempo modifier.

*/

#define F_CPU 8000000

#include <util/delay.h>

//button interrupt
volatile bool inputButtonValue;

//button press control
bool readingButton = false;
int button_delay;
int button_delay_b;
bool beenDoubleClicked = false;
bool beenLongPressed = false;
byte additionalClicks = 0;

int Scale[] = { 
0,13717,14532,15397,16312,17282,0,18310,19398,20552,21774,23069,24440,25894,0};

// Note buffer
const int Silence = 0;
volatile unsigned int Acc[] = {Silence, Silence, Silence, Silence};
volatile unsigned int Freqs[] = {0, 0, 0, 0 };

//Globals persist throughout tune
volatile unsigned int GlobalTicks = 0;
int NextTick = 0;
int TunePtr = 0;
char Octave = 0, LastIndex = 0, Duration = 24;
int Index = 0;
int currentTune = 1;
bool repeatTune = false;

//Tempo Modifier
bool tempoChange;
char lastTempoChangeRead;
float const factor = 0.125;
float speedFactor;
int tempRead; 

//Tunes
int numberOfTunes = 8;

PROGMEM const char Clear[] = {"2,^(^^^)"}; //silences all the channels
PROGMEM const char musScale[] = {"12, 0:CDEFGABCDEFGABCbagfedcbagfed 24, c^"};
PROGMEM const char Bach[] = {//BACH, Choral BWV153-9 Ach Gott, Wie Manches Herzeleid
  "12,-1:C(-1:G0:E1:C)-1:C(-1:G0:E^)-1:E(0:C0:G1:C)-1:E(0:C0:G^)-1:c(0:E0:G1:C)-1:c(0:E0:G^) 24,-1:F(0:c0:f0:a)-1:F(0:D0:G0:B)-1:e(0:E0:G1:C)-1:+F(0:d0:A1:D)-1:+F(0:d0:A1:c)-1:G(0:d0:g0:b)-1:d(0:d0:+f0:a)-1:d(0:d0:+f0:a)-1:d(0:d0:+f0:a)^(^^^)"
  "24,-1:D(0:d0:+F0:A)-1:G(0:d0:G0:B)-1:e(0:E0:G1:C)-2:b(0:d0:G1:D)-2:b(0:d0:G1:D)-1:C(0:E0:G1:c)-1:G(0:d0:G0:b)-1:d(0:d0:G0:a)12,-1:d(0:d0:+F0:a)-1:d(0:c0:+F0:a)24,-2:g(-1:b0:g0:g)-2:g(-1:b0:g0:g)-2:g(-1:b0:g0:g)^(^^^)"
  "12,0:C(0:C0:G1:E)0:C(0:C0:G^) -1:+G(-1:b0:e1:E)-1:+G(-1:b0:e^) -1:e(0:E0:+G1:E)-1:e(0:E0:+G^) 24, -1:A(0:E0:A1:c)-1:A(0:F0:B1:D)-2:a(0:G1:+C1:E)-1:D(0:f1:D1:F)-1:E(0:G1:c1:e)-1:F(0:c1:c1:e)-1:G(-1:b0:G1:d)-1:G(-1:b0:G1:d)-1:G(-1:b0:G1:d)^(^^^)"
  "24,-1:E(0:C0:G1:C)-1:d(-1:f0:B1:D)-1:c(-1:G0:C1:E)-1:F(-1:A0:C1:d)-1:G(-1:B0:b1:d) 32, -1:A(0:C0:e1:c) 4, ^(^^^) 24,-1:+f(0:d0:A1:c)-1:G(0:d0:g0:b)-2:g(0:d0:g0:b)-1:C(0:E0:G1:C)-1:C(0:E0:G1:C)-1:C(0:E0:G1:C)^(^^^)"};
PROGMEM const char Tuplets[] = {
  "24,0:C(G)D(A) 12,0:C(G)D(A)0:C(G)D(A) 8,0:C(G)D(A)0:C(G) D(A)0:C(G)D(A) 6,0:E(B)^(^)E(B)^(^)E(B)^(^)E(B)^(^) ^(^)"};
PROGMEM const char Tetris[] = {
  "24,0:G 12,d-E 24,F 12,-ed | 24,c 12,c-E 24,G 12,f-e 24,d 12,d-E 24,FG-ec 6,c^ 12,cD-E | 6,F^ 12,FF-A 24,C 12,-b-a 24,g 12,g-e 24,G 12,f-e 24,d 12,d-E 24,FG-ecc^"};
PROGMEM const char Tuplets1[] = { //tuplets in one voice only
  "4,-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:G)-1:C(0:E0:A)-1:C(0:E0:A)-1:C(0:E0:A)-1:C(0:E0:A)-1:G(0:F0:A)-1:G(0:F0:A)-1:G(0:F0:A)-1:G(0:F0:A)-1:G(0:F0:B)-1:G(0:F0:B)-1:G(0:F0:B)-1:G(0:F0:B)-1:G(0:F0:B)-1:G(0:F0:B)-1:G(0:F0:B)-1:G(0:F0:B)"
  "48,0:C(0:E1:C) -1:F(0:F1:C) -1:C(0:e1:C) ^(^^)"};
PROGMEM const char Madrigal[] = {//Madrigal in 4
  "48, 0:C(EGC) f(0:FAC)| d(0:FAD) d(0:+FAD) | G(0:G-BD) 24, -2:g(0:G-B-E) g(0:GC-E)|48, D(0:+FD^) 24, -0:D(^^)c |"
  "48, b(GDG) 24, -b(DGD) -b(EGC) 48, -a(FCF) 24, f(C-AC) f(D-AC) | 48, G(0:-EGC) 24, -2:g(0:dGC) g(0:dGB) 62, C(0:EGC)^(^^^)"};
PROGMEM const char Maria[] = {//Santa Maria Estrela do día
  "24,0:G 12,f-e 24,d 12,d-E 24,FGc 12,c^ | 24,G 12,f-e 24,d 12,cD-Edc-b 24,C 12,C^ |"
  "24,FGA 12,Af-BagA 24,f 12,f^ | 24,FGA 12,Af-Bagf 24,G 12,G^ | 24,A-BC 12,Ca-BC 6,-ba 12,g 24,A 12,A^ |12,A^A^ 24,A 12,Af-BagA 24,f 12,f^"
  "24,0:G 12,f-e 24,d 12,d-E 24,FGc 12,c^ | 24,G 12,f-e 24,d 12,cD-Edc-b 24,C 12,C^"};
PROGMEM const char Luke[] = {
  "24,0: C(-1:C)Fe 12,dE | 24,F(-1:D) 12,eF 24,A(-1:G)g | 12,f(-1:F)eF(-1:E)G 24,f(-1:D)e(-1:C) |"
  "12,d(-2:B)Edc 24,b(-2:G)G | c(-1:C)Fe 12,dE | 24,F(-1:D) 12,eF 24,A(-1:G)g |"
  "12,A(-1:F)gA(-1:E)Cb(-1:G)gB(-1:B)D | 24,c(-1:C)g 48,C ^(^^)"};

void setup() {
  
  PRR = (1 << PRUSI);                  //disable USI to save power as we are not using it
  DIDR0 = (1 << ADC1D) | (1 << ADC3D); //PB2,PB3  //disable digital input in pins that do analog conversion
  
  pinMode(0, OUTPUT); //LED
  pinMode(4, OUTPUT); //timer 1 in digital output 4 - outs 1 and 2
  pinMode(3, INPUT);  //analog- freq input (knob plus external input 1)
  pinMode(2, INPUT);  //analog- input (external input 2)
  pinMode(1, INPUT);  //digital input (push button)
  
  checkVoltage();
  ADMUX = 0;                           //reset multiplexer settings
  ADMUX |= (1<<ADLAR);                                  //left-adjust result (8 bit conversion) 
  ADMUX |= (1 << MUX1) | (1 << MUX0); 
  ADCSRA |= (1 << ADSC);                                //start conversion
  
  //set clock source for PWM -datasheet p94
  PLLCSR |= (1 << PLLE);               // Enable PLL (64 MHz)
  _delay_us(100);                      // Wait for a steady state
  while (!(PLLCSR & (1 << PLOCK)));    // Ensure PLL lock
  PLLCSR |= (1 << PCKE);               // Enable PLL as clock source for timer 1   
  
  cli();                               // Interrupts OFF (disable interrupts globally)
  
  //PWM Generation -timer 1
  GTCCR = (1 << PWM1B) | (2 << COM1B0);  // PWM B, clear on match
  OCR1C  = 0xff;                         // 255
  TCCR1  = (1 << CS10);                  // no prescale
  
  //Timer Interrupt Generation -timer 0
  TCCR0A = (1 << WGM01) | (1 << WGM00);  // fast PWM
  TCCR0B = (1 << WGM02) | (2 << CS00);   // 1/8 prescale
  OCR0A = 49;                            // Divide by 400 - 20kHz
  TIMSK = (1 << OCIE0A);                 // Enable interrupt on compare match
 
  //Watchdog Interrupt Generation -ticks timer.
  WDTCR = (1 << WDIE) | (0 << WDP0);     // 64 Hz 
  
  //Pin interrupt Generation
  GIMSK |= (1 << PCIE);                  // Enable Pin Change Interrupt 
  PCMSK |= (1 << PCINT1);                // on pin 01 
  
  sei();                                 // Interrupts ON (enable interrupts globally)
  
  resetTempoModifierParams();
  digitalWrite(0, HIGH); //lights ON
}

// Watchdog interrupt counts ticks (1/64 sec)
ISR(WDT_vect) {
  setTempoModifier();                                       //start next conversion
  if (!readingButton)
  GlobalTicks++;
}

ISR(PCINT0_vect) {                       //PIN Interruption - has priority over COMPA; this ensures that the switch will work
  inputButtonValue = PINB & 0x02;        // Reads button (digital input1, the second bit in register PINB. We check the value with & binary 10, so 0x02) 
}

// Generate triangle waves on 4 channels
ISR(TIMER0_COMPA_vect) { 
  volatile signed char Mask, Temp, Sum;   //change: marking the vars as volatile got rid of all the clicks when repeating the same note
  for (int c=0; c<=3; c++) {
    Acc[c] = Acc[c] + Freqs[c];
    Temp = Acc[c] >> 8;
    Mask = Temp >> 7;
    Sum = Sum + ((char)(Temp ^ Mask) >> 1);
  }
  OCR1B = Sum;
}

void loop(){
  checkButton();
 
  playList(currentTune); 
}

void setTempoModifier(){ //a modifier to the tune's base tempo 
  tempRead = 16 - (ADCH >> 4); //values between 16 and 1 -- the highest the reading, the slowest the final tempo
  ADCSRA |=  (1<<ADSC);   
  if(!tempoChange) {
    if(tempRead == lastTempoChangeRead) {      
      tempoChange = true;
    }
  }
  else {
    lastTempoChangeRead = tempRead;
    speedFactor = tempRead * factor;    //the smaller the factor, the faster the speed
  }
}

void checkButton() {
  while (inputButtonValue == HIGH) {
    if (!beenLongPressed) digitalWrite(0, LOW);      //so when the LED turns off we know for sure that the input has been acknowledged. After a while pressing,it will turn on again so that we know that the long press was acknowledged.
    readingButton = true;
    button_delay++;
    _delay_ms(10);
    if (button_delay > 20 &! beenDoubleClicked) {
      digitalWrite(0, HIGH);
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
        
        if (button_delay_b == 50) {
          if (additionalClicks == 0){           
            if (beenLongPressed) {                    //button released after being pressed for a while
              readingButton = false;
              repeatTune = !repeatTune;
              
              beenLongPressed = false;
              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
            else {                                    //button released (regular single click)
              readingButton = false;
              flashLEDOnce();
              advancePlaylist();

              button_delay = 0;
              button_delay_b = 0;
              additionalClicks = 0;
              hold = false;
            }
          }
          else if (additionalClicks == 1) {          //more than one click    
            readingButton = false; 
            flashLEDTwice();    
            rewind();
            
            button_delay = 0;
            button_delay_b = 0;
            additionalClicks = 0;
            beenDoubleClicked = true;
            hold = false;
          }
           else if (additionalClicks > 1 ) {                 //button pressed at least twice more(triple click or more)
            readingButton = false;
            flashLEDTwice();    
            rewindAndStop();

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

void playList (int tune) { //change: playlist
  switch (tune) {
    case 0:
      play(Clear);
      break;
    case 1:
      play(musScale);
      break;
    case 2:
      play(Luke);
      break;
    case 3:
      play(Tuplets);
      break;
    case 4:
      play(Tuplets1);
      break;
    case 5:
      play(Maria);
      break;
    case 6:
      play(Bach);
      break;
    case 7:
      play(Tetris);
      break;
    case 8:
      play(Madrigal);
      break;
  }
}

void advancePlaylist(){
    repeatTune = false;
    resetTempoModifierParams();
    rewind();
    play(Clear);
    rewind();
    currentTune++;
    if (currentTune > numberOfTunes) currentTune = 1; //we skip tune 0 because that's a special tune that resets all the channels
}

void rewindAndStop(){
    repeatTune = false;
    resetTempoModifierParams();
    rewind();
    play(Clear);
    rewind();
    currentTune = 0;
}

void rewind(){  //called from play() when we set a tune to repeat (that's why it doesn't have repeatTune = false) 
  NextTick = 0;
  GlobalTicks = 0; 
  TunePtr = 0;
  Index = 0;
  LastIndex = 0;
}

void resetTempoModifierParams(){
tempoChange = false;
lastTempoChangeRead = 4;                          //a reading from the potentiometer
speedFactor = 1;                                  //the actual modifier
}

// Parse Ample tune notation
void play(const char theTune[]) { 
  char Sign = 0, Number = 0;
  char Symbol, Chan, SaveIndex, SaveOctave;
  boolean More = 1, ReadNote = 0, Bra = 0, SetOctave = 0;
  do {
    do { // Skip formatting characters
      Symbol = pgm_read_byte(&theTune[TunePtr++]);
    } while ((Symbol == ' ') || (Symbol == '|'));
    char CapSymbol = Symbol & 0x5F;
    if (Symbol == '(') { Bra = 1; SaveIndex = LastIndex; SaveOctave = Octave;}
    else if (ReadNote && !Bra) More = 0; 
    else if (Symbol == ')') { Bra = 0; LastIndex = SaveIndex; Octave = SaveOctave; }
    else if (Symbol == 0) // End of string - stop
    {
      if (!repeatTune) break; 
      rewind();
    }
    else if (Symbol == ',') { Duration = Number; Number = 0; Sign = 0; }
    else if (Symbol == ':') {
      SetOctave = 1; Octave = Number;
      if (Sign == -1) Octave = -Octave;
      Number = 0; Sign = 0;
    }
    else if ((Symbol >= '0') && (Symbol <= '9')) Number = Number*10 + Symbol - '0';
    else if (Symbol == '<') Octave--;
    else if (Symbol == '>') Octave++;
    else if (Symbol == '-') Sign = -1;
    else if (Symbol == '+') Sign = 1;
    else if (Symbol == '/') ReadNote = 1;
    else if (Symbol == '^') { attenuate(Acc[Chan]); Freqs[Chan++] = 0; ReadNote = 1;  } //change:attenuate rather than silence (solves clicks)
    else if ((CapSymbol >= 'A') && (CapSymbol <= 'G')) {
      boolean Lowercase = (Symbol & 0x20);
      Index = (((CapSymbol - 'A' + 5) % 7) << 1) + 1 + Sign;
      if (!SetOctave) {
        if (LastIndex && (Index < LastIndex) && !Lowercase) Octave++;
        if (LastIndex && (Index > LastIndex) && Lowercase) Octave--;
      } else SetOctave = 0;
      Freqs[Chan++] = Scale[Index] >> (4 - Octave);
      LastIndex = Index;
      ReadNote = 1; Sign = 0;
    } 
  } while (More && (!readingButton));
  TunePtr--;
  NextTick = NextTick + (Duration * speedFactor);    //change: allows to modify the speed 
  do ; while (Ticks() < NextTick);
  
}

void attenuate(int chan){      //change: attenuate gradually (solves clicks)
  while (chan > 0){  
  --chan;
  }
}

// Ticks timer
unsigned int Ticks() {
  unsigned long t;
  uint8_t oldSREG = SREG;  
  t = GlobalTicks;
  SREG = oldSREG;
  return t;
}

void flashLEDOnce () {  
  digitalWrite(0, LOW);  
  _delay_ms(30);
  digitalWrite(0, HIGH);
}

void flashLEDTwice () {
  digitalWrite(0, HIGH); 
  _delay_ms(30);
  digitalWrite(0, LOW);
  _delay_ms(30);
  digitalWrite(0, HIGH); 
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
