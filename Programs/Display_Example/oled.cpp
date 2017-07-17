/*
//******************************
//*   miniMO OLED_SSD1306      *
//*   2017 by enveloop         *
//******************************
  Adapted from the SSD1306xLED library 
        by Neven Boyanov,
        as described Here:
  https://tinusaur.org/projects/ssd1306xled/
  SCROLL DOWN FOR SSD1306xLED LICENSE

//
   http://www.minimosynth.com/
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
//

*/

#include "oled.h"

#define sbi(bit) PORTB |= (1 << (bit))  //macro to set a bit in PORTB
#define cbi(bit) PORTB &= ~(1 << (bit)) //macro to clear a bit in PORTB

#define HOR_OFFSET      (128 - SCREEN_WIDTH) / 2

void OLED::init(void) 
{
  DDRB |= (1 << SDA);	                       //set port as output
  DDRB |= (1 << SCL);	                       //set port as output

  send_command(0xAE);                          //display off
  send_command(0xD5); send_command(0x80);      //0xD5 command to set display clock // 0x80 recommended value
  
  if (SCREEN_HEIGHT == 32)
  {
    send_command(0xA8); send_command(0x1F);    //set multiplex: 0x1F for height = 32, 0x3F for  height = 64
  }
  else send_command(0xA8); send_command(0x3F);
                                       
  send_command(0x40);                          //zero start line
  send_command(0x8D); send_command(0x14);      //charge pump
   
  send_command(0x20); send_command(0x02);      //0x20 command to set memory mode: 0x02, page addressing

  if (SCREEN_HEIGHT == 32)
  {
    send_command(0xDA); send_command(0x02);    //0xDA set com pins
  }
  else send_command(0xDA); send_command(0x12); 
  
  send_command(0xD3); send_command(0x00);      //0xD3 command to set display offset (vertical)
  send_command(0xA6);                          //command to set Display color 0xA6 Normal 0xA7=Inverse
  
  send_command(0x81); send_command(0x7F);      //0x81 command to send contrast, 7F actual value (0x00 to 0xFF)
  
  if (FLIPSCREEN) 
  {
    send_command(0xA1);                        //0xA0/0xA1 flip horizontally
    send_command(0xC8);                        //0xC0/0xC8 flip vertically
  }

  send_command(0xD9); send_command(0xF1);      //0xD9 command to set pre charge
   
  send_command(0xDB); send_command(0x80);      //set vcom detect
   
  send_command(0xA4);                          //entire display ON, follow RAM
  send_command(0xAF);                          //display ON
  
  fillscreen(0x00);                            //clear screen
}

void OLED::xfer_start(void)
{
  sbi(SCL);	
  sbi(SDA);     
  cbi(SDA);	
  cbi(SCL);	
}

void OLED::xfer_stop(void) 
{
  cbi(SCL);        
  cbi(SDA);	   
  sbi(SCL);	   
  sbi(SDA);        
}

void OLED::send_byte(uint8_t byte) 
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if ((byte << i) & 0x80) sbi(SDA); 
    else cbi(SDA); 
    sbi(SCL); 
    cbi(SCL);
  }
  sbi(SDA); 
  sbi(SCL); 
  cbi(SCL);
}

void OLED::send_command(uint8_t command) 
{
  xfer_start(); 	
  send_byte(SA);          // slave address
  send_byte(0x00);	  // write command 	
  send_byte(command); 	
  xfer_stop(); 
} 

void OLED::send_data_start(void)
{ 	
  xfer_start(); 	
  send_byte(SA);          // slave address	
  send_byte(0x40);	  // write data 
} 

void OLED::send_data_stop(void)
{ 	
  xfer_stop(); 
} 

void OLED::setpos(unsigned int x, unsigned int y) 
{ 
  xfer_start(); 	
  send_byte(SA);                  //slave address 	
  send_byte(0x00);	          //write command 	
  send_byte(0xb0 + y); 	
  send_byte((((x + HOR_OFFSET - 1) & 0xf0) >> 4) | 0x10);  // |0x10
  send_byte(((x + HOR_OFFSET - 1) & 0x0f) | 0x01);         // |0x01
  xfer_stop();
}

void OLED::fillscreen(unsigned int fill_Data)
{
  uint8_t k, m, n;
  k = SCREEN_HEIGHT >> 3;       //screenHeight / 8
  for (m = 0; m < k; m++)              
  {
    send_command(0xb0 + m);	//page0-page1
    send_command(0x00);		//low column start address
    send_command(0x10);		//high column start address
    send_command(0x21); send_command(HOR_OFFSET); send_command(127 - HOR_OFFSET); //0x21 command to setup column start and end address. Values are decimal
    send_data_start();
    for (n = 0; n < SCREEN_WIDTH; n++) 		
    {
      send_byte(fill_Data);
    } 		
    send_data_stop();
  }
} 

void OLED::fillLine(unsigned int line, unsigned int fill_Data)
{
  uint8_t n;
    send_command(0xb0 + line);	//page0-page1
    send_command(0x00);		//low column start address
    send_command(0x10);		//high column start address
    send_command(0x21); send_command(HOR_OFFSET); send_command(127 - HOR_OFFSET); //0x21 command to setup column start and end address. Values are decimal
    send_data_start();
    for (n = 0; n < SCREEN_WIDTH; n++) 		
    {
      send_byte(fill_Data);
    }
    send_data_stop();
} 

void OLED::char_f6x8(uint8_t x, uint8_t y, const char ch[])
{
  uint8_t c, i, j = 0; 	
  while (ch[j] != '\0') 	
  {
    c = ch[j] - 32; 	
    
    //without this check, if the characters are longer than the size of the screen, they wrap up and overwrite the line
    if (x > (SCREEN_WIDTH - 2))          //orig 126  --screenwidth - 2
    {
      x = 0;
      y++;
    }
    setpos(x, y);
    send_data_start();
    for (i = 0; i < 6; i++)
    {
      send_byte(pgm_read_byte(&font6x8[c * 6 + i]));
    }
    send_data_stop();
    x += 6;
    j++;
  }
}

/*
//SSD1306xLED LICENSE//
Copyright (c) 2016 Neven Boyanov, Tinusaur Team. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy 
of this software and associated documentation files (the "Software"), to deal 
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included 
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
THE SOFTWARE.
*/

