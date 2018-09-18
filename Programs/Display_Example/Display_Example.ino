/*
//******************************
//*   miniMO DISPLAY EXAMPLE   *
//*   2017-18 by enveloop      *
//******************************

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

CONCEPT

This is an example program to show how to work with OLED SSD1306 I2C Screens.
The program reads inputs attached to I/O 3, shows the readings on the screen,
and toggles the outputs ON or OFF depending on the value; 
this last behavior will work even if the screen is not connected. 
The program also detects button clicks, and shows a text accordingly. 

I/O
  1&2: Outputs (ON or OFF)
  3: Input
  4: not used

SCREEN SETUP - HARDWARE

  -Locate miniMO's programming header, under the LED light
    -Note the vertical writing next to the pins, from bottom to top: GND, SCK, MI, MO, RST, VCC 
    -Connect the Screen's VCC to the header's VCC
    -Connect the screen's GND to the header's GND
    -Connect the screen's SCL to miniMO's MO
    -Connect the screen's SDA to miniMO's SCK
  -With the screen connected, 
    -Load a battery in miniMO's holder
    -Connect all three pins of the Battery-External male header AT ONCE
      -The easiest way to do this is to get a three pin female header and wire or solder its terminals together

SCREEN SETUP - SOFTWARE

  In oled.h , set the width, height, and orientation of your screen

OPERATION

  Knob: Adjust the readings according to the input. Range: 0 - 255
    -If you'd prefer a wider range (0 to 1023), simply remove " >>2 " from line 79
  Click: Shows the text "PUSH" on screen while clicking

*/

#include <avr/io.h>
#include "oled.h"

OLED oled = OLED();
byte valueDisplayLine = 3;
char inputChar[] = "0000";

//external input smoothing
const byte numReadings = 5;     //smoothing factor
int readings[numReadings];      // stores readings from the analog input
byte readIndex = 0;             // the index of the current reading
int total = 0;                  // the running total
static int smoothRead;

void setup() 
{
  delay(40);
  oled.init();
  oled.fillscreen(0x00);                          // clear screen
  oled.char_f6x8(0, 0, "INPUT");
  oled.char_f6x8(0, 1, "READING");
  delay(100);
  
  pinMode(1, INPUT);  //Digital input (push button)
  pinMode(3, INPUT);  //Analog input - potentiometer and I/O 3
  pinMode(4, OUTPUT); //Outputs 1 and 2 
  
}

void loop() 
{                           
  smoothRead = inputRead(3);                       //take an averaged reading and store it in a variable
  itoa(smoothRead, inputChar, 10);                 //convert the stored reading, in base 10, to ascii, and store the result it in var inputChar
  oled.fillLine(valueDisplayLine,0x00);            //clear the line
  oled.char_f6x8(10, valueDisplayLine, inputChar); //print the reading
  
  if (smoothRead > 200) digitalWrite(4, 255);      //activate the output depending on the reading
  else digitalWrite(4, 0);
  
  if (digitalRead(1) == HIGH) 
  {
    oled.char_f6x8(30, 0, "PUSH");
    //tone(4, 440, 10);
  }
  else
  {
    oled.char_f6x8(30, 0, "    ");
  }
  delay(500);                                       //interval between readings
}

int inputRead(byte pin) {                     //with averaging
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(pin) >> 2; // right shifting by 2 to get values between 0 and 255 (0-1023/2^2)
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) readIndex = 0;
  return total / numReadings;
}
