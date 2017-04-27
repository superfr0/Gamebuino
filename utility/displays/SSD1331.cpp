/*
 * (C) Copyright 2014 Aurélien Rodot. All rights reserved.
 *
 * This file is part of the Gamebuino Library (http://gamebuino.com)
 *
 * The Gamebuino Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *  
 * Parts of the graphical library come from the great library provided by Adafruit
 * for their Nokia 5110 module which can be found here :
 * https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library
 * Here is their license :
 * 
 * This is the core graphics library for all our displays, providing a common
 * set of graphics primitives (points, lines, circles, etc.). It needs to be
 * paired with a hardware-specific library for each display device we carry
 * (to handle the lower-level functions).
 * Adafruit invests time and resources providing this open source code, please
 * support Adafruit & open-source hardware by purchasing products from Adafruit!
 * Copyright (c) 2013 Adafruit Industries. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * Portions of this code taken from Adafruit SD1331 Driver
 *
 * All text above, and the splash screen below must be included in any redistribution
 */

#include "Display.h"

// a 3x5 font table
extern const uint8_t font3x5[] PROGMEM;

// the memory buffer for the LCD
uint8_t _displayBuffer[LCDHEIGHT_NOROT * LCDWIDTH_NOROT / 8];

void Display::begin(int8_t SCLK, int8_t DIN, int8_t DC, int8_t CS, int8_t RST)
{
    din = DIN;
    sclk = SCLK;
    dc = DC;
    rst = RST;
    cs = CS;

    fontSize = 1;
    color = BLACK;
	bgcolor = WHITE;
    textWrap = true;
	setFont(font3x5);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
	/* DIV2 is the fastest setting, this seems to work just fine for the SSD1331 and SSD1332 */
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setDataMode(SPI_MODE3);
	
    // set pin directions	
	pinMode(din, OUTPUT);
	pinMode(sclk, OUTPUT);
    pinMode(dc, OUTPUT);
	
    if(rst > 0)
	{
        pinMode(rst, OUTPUT);
	}
	
    if(cs > 0)
	{
        pinMode(cs, OUTPUT);
	}

    // toggle RST low to reset
    if(rst > 0)
	{
        digitalWrite(rst, HIGH);
        delay(500);
        digitalWrite(rst, LOW);
        delay(500);
        digitalWrite(rst, HIGH);
        delay(500);
    }

    clkport = portOutputRegister(digitalPinToPort(sclk));
    clkpinmask = digitalPinToBitMask(sclk);	
    mosiport = portOutputRegister(digitalPinToPort(din));
    mosipinmask = digitalPinToBitMask(din);
    csport = portOutputRegister(digitalPinToPort(cs));
    cspinmask = digitalPinToBitMask(cs);
    dcport = portOutputRegister(digitalPinToPort(dc));
    dcpinmask = digitalPinToBitMask(dc);
	
    command(CMD_DISPLAYOFF);    // 0xAE
    command(CMD_SETREMAP);      // 0xA0
#if defined COLORORDER_RGB
    command(0xF2);              // 8 bit RGB mode
//    command(0x72);				// 16bit RGB Color
#else
    command(0x36);              // 8 bit BGR mode
//    command(0x76);				// 16bit BGR Color
#endif
    command(CMD_STARTLINE);     // 0xA1
    command(0x0);
    command(CMD_DISPLAYOFFSET); // 0xA2
    command(0x0);
    command(CMD_NORMALDISPLAY); // 0xA4
    command(CMD_SETMULTIPLEX);  // 0xA8
    command(0x3F);              // 0x3F 1/64 duty
    command(CMD_SETMASTER);     // 0xAD
    command(0x8E);
    command(CMD_POWERMODE);     // 0xB0
    command(0x0B);
    command(CMD_PRECHARGE);     // 0xB1
    command(0x31);
    command(CMD_CLOCKDIV);      // 0xB3
    command(0xF0);              // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    command(CMD_PRECHARGEA);    // 0x8A
    command(0x64);
    command(CMD_PRECHARGEB);    // 0x8B
    command(0x78);
    command(CMD_PRECHARGEA);    // 0x8C
    command(0x64);
    command(CMD_PRECHARGELEVEL);// 0xBB
    command(0x3A);
    command(CMD_VCOMH);         // 0xBE
    command(0x3E);
    command(CMD_MASTERCURRENT); // 0x87
    command(0x06);
    command(CMD_CONTRASTA);     // 0x81
    command(0x91);
    command(CMD_CONTRASTB);     // 0x82
    command(0x50);
    command(CMD_CONTRASTC);     // 0x83
    command(0x7D);
    command(CMD_DISPLAYON);     //--turn on oled panel    
	
    update();
}

void Display::command(uint8_t c)
{
    noInterrupts();
	
    *dcport &= ~ dcpinmask;
    *csport &= ~ cspinmask;
    
	SPI.transfer(c);
    
    *csport |= cspinmask;
	
    interrupts();
}

void Display::data(uint8_t c)
{
    noInterrupts();
	
    *dcport |= dcpinmask;    
    *csport &= ~ cspinmask;
    
    SPI.transfer(c);
    
    *csport |= cspinmask;
    interrupts();
}

void Display::setContrast(uint8_t val)
{
	contrast = constrain(val, 30, 80);
	/* I'm not sure if we have a concept of contrast on SSD1331/1332 */
}

void Display::clear(void)
{
    memset(_displayBuffer, 0, LCDWIDTH_NOROT * LCDHEIGHT_NOROT / 8);
    cursorY = cursorX = 0;
}

void Display::fillScreen(uint8_t color)
{
    memset(_displayBuffer, 0xFFFF, LCDWIDTH_NOROT * LCDHEIGHT_NOROT / 8);
}

void Display::update(void)
{
    static int c = 0;
    uint8_t x = 0;
    uint8_t y = 0;

    frameCount ++;

    /* Set top left */
    command(CMD_SETCOLUMN);
    command(x);
    command(LCDWIDTH-1);

    command(CMD_SETROW);
    command(y);
    command(LCDHEIGHT-1);

    *dcport |= dcpinmask;

    *csport &= ~ cspinmask;

    /* Since the data format is optimized for the PCD8544 stream, we'll parse the x/y and fill them in linearly */
    for(y = 0; y < LCDHEIGHT; y++)
    {
        for(x=0; x < LCDWIDTH; x++)
        {
    #ifdef COLOR_SCREEN_OVERRIDE
            SPI.transfer( ((_displayBuffer[ x + (y / 8) * LCDWIDTH] >> (y % 8 )) & 0x1) ? COLOR_SCREEN_OVERRIDE : BLACK );
    #elif COLOR_SCREEN_OVERRIDE_CYCLE
        if((_displayBuffer[ x + (y / 8) * LCDWIDTH] >> (y % 8 )) & 0x1)
            SPI.transfer ( y + 70 +(c));
        else
            SPI.transfer ( 0x0 );
    #else
            SPI.transfer( ((_displayBuffer[ x + (y / 8) * LCDWIDTH] >> (y % 8 )) & 0x1) ? WHITE : BLACK );
    #endif
        }
    }

    #ifdef COLOR_SCREEN_OVERRIDE_CYCLE	
    c ++;
    #endif	

    *csport |= cspinmask;
}

void Display::setColor(int8_t c)
{
	color = c;
	bgcolor = c;
}

void Display::setColor(int8_t c, int8_t bg)
{
	color = c;
	bgcolor = bg;
}

void Display::drawFastVLine(int8_t x, int8_t y, int8_t h)
{
	for(int8_t i=0; i<h; i++)
	{
		drawPixel(x,y+i);
	}
}

void Display::drawFastHLine(int8_t x, int8_t y, int8_t w)
{
    for(int8_t i=0; i<w; i++)
	{
		drawPixel(x+i,y);
	}
}

void Display::drawRect(int8_t x, int8_t y, int8_t w, int8_t h)
{
    drawFastHLine(x, y, w);
    drawFastHLine(x, y + h - 1, w);
    drawFastVLine(x, y, h);
    drawFastVLine(x + w - 1, y, h);
}

void Display::fillRect(int8_t x, int8_t y, int8_t w, int8_t h)
{
    // stupidest version - update in subclasses if desired!
    for(int8_t i = y; i < y + h; i++)
	{
        drawFastHLine(x, i, w);
    }
}

void Display::drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1)
{
    int8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if(steep)
	{
        swap(x0, y0);
        swap(x1, y1);
    }

    if(x0 > x1)
	{
        swap(x0, x1);
        swap(y0, y1);
    }

    int8_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int8_t err = dx / 2;
    int8_t ystep;

    if(y0 < y1)
	{
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for(; x0 <= x1; x0++)
	{
        if(steep)
		{
            drawPixel(y0, x0);
        }
        else
        {
            drawPixel(x0, y0);
        }
        
        err -= dy;
        
        if(err < 0)
		{

            y0 += ystep;
            err += dx;
        }
    }
}

void Display::drawCircle(int8_t x0, int8_t y0, int8_t r)
{
    int8_t f = 1 - r;
    int8_t ddF_x = 1;
    int8_t ddF_y = -2 * r;
    int8_t x = 0;
    int8_t y = r;

    drawPixel(x0, y0 + r);
    drawPixel(x0, y0 - r);
    drawPixel(x0 + r, y0);
    drawPixel(x0 - r, y0);

    while (x < y)
	{
        if(f >= 0)
		{
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        
        x++;
        ddF_x += 2;
        f += ddF_x;

        drawPixel(x0 + x, y0 + y);
        drawPixel(x0 - x, y0 + y);
        drawPixel(x0 + x, y0 - y);
        drawPixel(x0 - x, y0 - y);
        drawPixel(x0 + y, y0 + x);
        drawPixel(x0 - y, y0 + x);
        drawPixel(x0 + y, y0 - x);
        drawPixel(x0 - y, y0 - x);
    }
}

void Display::drawCircleHelper(int8_t x0, int8_t y0, int8_t r, uint8_t cornername)
{
    int8_t f = 1 - r;
    int8_t ddF_x = 1;
    int8_t ddF_y = -2 * r;
    int8_t x = 0;
    int8_t y = r;

    while (x < y)
	{
        if(f >= 0)
		{
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        if(cornername & 0x4)
		{
            drawPixel(x0 + x, y0 + y);
            drawPixel(x0 + y, y0 + x);
        }
        
        if(cornername & 0x2)
		{
            drawPixel(x0 + x, y0 - y);
            drawPixel(x0 + y, y0 - x);
        }
        
        if(cornername & 0x8)
		{
            drawPixel(x0 - y, y0 + x);
            drawPixel(x0 - x, y0 + y);
        }
        
        if(cornername & 0x1)
		{
            drawPixel(x0 - y, y0 - x);
            drawPixel(x0 - x, y0 - y);
        }
    }
}

void Display::fillCircle(int8_t x0, int8_t y0, int8_t r)
{
    drawFastVLine(x0, y0 - r, 2 * r + 1);
    fillCircleHelper(x0, y0, r, 3, 0);
}

// used to do circles and roundrects!
void Display::fillCircleHelper(int8_t x0, int8_t y0, int8_t r, uint8_t cornername, int8_t delta)
{
    int8_t f = 1 - r;
    int8_t ddF_x = 1;
    int8_t ddF_y = -2 * r;
    int8_t x = 0;
    int8_t y = r;

    while (x < y)
	{
        if(f >= 0)
		{
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        
        x++;
        ddF_x += 2;
        f += ddF_x;

        if(cornername & 0x1)
		{
            drawFastVLine(x0 + x, y0 - y, 2 * y + 1 + delta);
            drawFastVLine(x0 + y, y0 - x, 2 * x + 1 + delta);
        }
        
        if(cornername & 0x2)
		{
            drawFastVLine(x0 - x, y0 - y, 2 * y + 1 + delta);
            drawFastVLine(x0 - y, y0 - x, 2 * x + 1 + delta);
        }
    }
}

void Display::drawRoundRect(int8_t x, int8_t y, int8_t w,int8_t h, int8_t r)
{
    // smarter version
    drawFastHLine(x + r, y, w - 2 * r); // Top
    drawFastHLine(x + r, y + h - 1, w - 2 * r); // Bottom
    drawFastVLine(x, y + r, h - 2 * r); // Left
    drawFastVLine(x + w - 1, y + r, h - 2 * r); // Right
    // draw four corners
    drawCircleHelper(x + r, y + r, r, 1);
    drawCircleHelper(x + w - r - 1, y + r, r, 2);
    drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4);
    drawCircleHelper(x + r, y + h - r - 1, r, 8);
}

void Display::fillRoundRect(int8_t x, int8_t y, int8_t w,int8_t h, int8_t r)
{
    fillRect(x + r, y, w - 2 * r, h);
    // draw four corners
    fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1);
    fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1);
}

void Display::drawTriangle(int8_t x0, int8_t y0, int8_t x1, int8_t y1, int8_t x2, int8_t y2)
{
    drawLine(x0, y0, x1, y1);
    drawLine(x1, y1, x2, y2);
    drawLine(x2, y2, x0, y0);
}

void Display::fillTriangle(int8_t x0, int8_t y0, int8_t x1, int8_t y1, int8_t x2, int8_t y2)
{
    int8_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if(y0 > y1)
	{
        swap(y0, y1);
        swap(x0, x1);
    }
    
    if(y1 > y2)
	{
        swap(y2, y1);
        swap(x2, x1);
    }
    
    if(y0 > y1)
	{
        swap(y0, y1);
        swap(x0, x1);
    }

    if(y0 == y2)
	{ // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)
        {
            a = x1;
        }
        else if(x1 > b)
        {
            b = x1;
        }
        
        if(x2 < a)
        {
            a = x2;
        }
        else if(x2 > b)
        {
            b = x2;
        }
        
        drawFastHLine(a, y0, b - a + 1);
        return;
    }

    int16_t
    dx01 = x1 - x0,
            dy01 = y1 - y0,
            dx02 = x2 - x0,
            dy02 = y2 - y0,
            dx12 = x2 - x1,
            dy12 = y2 - y1,
            sa = 0,
            sb = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2)
    {
        last = y1; // Include y1 scanline
    }
    else
    {
        last = y1 - 1; // Skip it
    }

    for(y = y0; y <= last; y++)
	{
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if(a > b)
        {
            swap(a, b);
        }
        
        drawFastHLine(a, y, b - a + 1);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    
    for(; y <= y2; y++)
	{
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;

        /* longhand:
                a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
                b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if(a > b)
        {
            swap(a, b);
        }
        
        drawFastHLine(a, y, b - a + 1);
    }
}

void Display::drawBitmap(int8_t x, int8_t y, const uint8_t *bitmap)
{
   int8_t w = pgm_read_byte(bitmap);
   int8_t h = pgm_read_byte(bitmap + 1);
   bitmap = bitmap + 2; //add an offset to the pointer to start after the width and height
#if(ENABLE_BITMAPS > 0)
/*   original code
    int8_t i, j, byteWidth = (w + 7) / 8;
    for(j = 0; j < h; j++)
	{
        for(i = 0; i < w; i++)
		{
            if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (B10000000 >> (i % 8)))
			{
                drawPixel(x + i, y + j);
            }
        }
    }
  */
  uint8_t * buffer = getBuffer();
  const uint8_t col = color;
  const uint8_t bw = (w+7) / 8;
  
  // clip
  if(x >= LCDWIDTH)
  {
    return;
  }
  
  if(x + w <= 0)
  {
    return;
  }
  
  if(y >= LCDHEIGHT)
  {
    return;
  }
  
  if(y + h <= 0)
  {
    return;
  }
  
  if(y < 0)
  {
    h += y, bitmap -= bw * y, y = 0;
  }
  
  if(y + h > LCDHEIGHT)
  {
    h = LCDHEIGHT - y;  
  }
   
  uint8_t x1 = max(0, x);
  uint8_t x2 = min(LCDWIDTH, x + w);
  
#ifdef ENABLE_GRAYSCALE
   uint8_t g = y ^ frameCount;
#endif  

  // draw
  uint8_t first_bitmap_mask = 0x80 >> ((x1 - x) & 7);
  const uint8_t * bitmap_line = bitmap + (x1 - x) / 8;
  uint8_t screen_mask = 0x01 << (y % 8);
  uint8_t * screen_row = buffer + (y / 8) * LCDWIDTH + x1;  
  for(uint8_t dy=0; dy<h; dy++, bitmap_line+=bw)
  {
    const uint8_t * bitmap_ptr = bitmap_line;    
    uint8_t bitmap_mask = first_bitmap_mask;    
    uint8_t pixels = pgm_read_byte(bitmap_ptr);
    uint8_t * dst = screen_row;
    
    if(col == BLACK)
	{
      for(uint8_t sx=x1; sx<x2; sx++, dst++)
      {
			if(pixels & bitmap_mask)
			{
			  *dst |= screen_mask;
			}
            
			bitmap_mask >>= 1;
			
			if(!bitmap_mask)
			{
				bitmap_mask = 0x80;
				pixels = pgm_read_byte(++bitmap_ptr);
			}
		}
	}
    else if(col == WHITE)
    {
      uint8_t inv_screen_mask = ~screen_mask;
	  
      for(uint8_t sx=x1; sx<x2; sx++, dst++)
      {
        if(pixels & bitmap_mask)
		{
			*dst &= inv_screen_mask;
		}
		
        bitmap_mask >>= 1;
		
        if(!bitmap_mask)
        {
			bitmap_mask = 0x80;
			pixels = pgm_read_byte(++bitmap_ptr);
        }
      }
    }
#ifdef ENABLE_GRAYSCALE
    else if(col == GRAY)
    {
      uint8_t inv_screen_mask = ~screen_mask;
      for(uint8_t sx=x1; sx<x2; sx++, dst++)
      {
        if(pixels & bitmap_mask)
        {
			if((sx^g) & 1)
			{
				*dst |= screen_mask;
			}
			else
			{
				*dst &= inv_screen_mask;
			}
        }
		
        bitmap_mask >>= 1;
		
        if(!bitmap_mask)
        {
          bitmap_mask = 0x80;
          pixels = pgm_read_byte(++bitmap_ptr);
        }
      }
       g ^= 1;
    }
#endif
   else // invert
   {
      for(uint8_t sx=x1; sx<x2; sx++, dst++)
      {
        if(pixels & bitmap_mask)
		{
          *dst ^= screen_mask;
		}

        bitmap_mask >>= 1;
		
        if(!bitmap_mask)
        {
          bitmap_mask = 0x80;
          pixels = pgm_read_byte(++bitmap_ptr);
        }
      }
    }
	
    screen_mask <<= 1;
	
    if(!screen_mask)
    {
      screen_mask = 1;
      screen_row += LCDWIDTH;
    }
  }
#else
   drawRect(x, y, w, h);
#endif
}

boolean Display::getBitmapPixel(const uint8_t* bitmap, uint8_t x, uint8_t y)
{
  return pgm_read_byte(bitmap+2 + y * ((pgm_read_byte(bitmap)+7)/8) + (x >> 3)) & (B10000000 >> (x % 8));
}

void Display::drawBitmap(int8_t x, int8_t y, const uint8_t *bitmap, uint8_t rotation, uint8_t flip)
{
    if((rotation == NOROT) && (flip == NOFLIP))
	{
        drawBitmap(x,y,bitmap); //use the faster algorithm
        return;
    }
	
    uint8_t w = pgm_read_byte(bitmap);
    uint8_t h = pgm_read_byte(bitmap + 1);
    bitmap = bitmap + 2; //add an offset to the pointer to start after the width and height
#if(ENABLE_BITMAPS > 0)
    int8_t i, j, //coordinates in the raw bitmap
            k, l, //coordinates in the rotated/flipped bitmap
            byteNum, bitNum, byteWidth = (w + 7) >> 3;

    rotation %= 4;

    for(i = 0; i < w; i++)
	{
        byteNum = i / 8;
        bitNum = i % 8;
		
        for(j = 0; j < h; j++)
		{
            if(pgm_read_byte(bitmap + j * byteWidth + byteNum) & (B10000000 >> bitNum))
			{
                switch (rotation)
				{
                    case NOROT: //no rotation
                        k = i;
                        l = j;
                        break;
                    case ROTCCW: //90° counter-clockwise
                        k = j;
                        l = w - i - 1;
                        break;
                    case ROT180: //180°
                        k = w - i - 1;
                        l = h - j - 1;
                        break;
                    case ROTCW: //90° clockwise
                        k = h - j - 1;
                        l = i;
                        break;
                }
				
                if(flip)
				{
                    flip %= 4;
					
                    if(flip & B00000001)
					{ //horizontal flip
                        k = w - k - 1;
                    }
					
                    if(flip & B00000010)
					{ //vertical flip
                        l = h - l;
                    }
                }
                k += x; //place the bitmap on the screen
                l += y;
                drawPixel(k, l);
            }
        }
    }
#else
    drawRect(x, y, w, h);
#endif
}

#if ARDUINO >= 100
size_t Display::write(uint8_t c)
{
#else
void Display::write(uint8_t c)
{
#endif

    if(c == '\n')
	{
        cursorY += fontSize * fontHeight;
        cursorX = 0;
    }
	else if(c == '\r')
	{
        // skip em
    }
	else
	{
        drawChar(cursorX, cursorY, c, fontSize);
        cursorX += fontSize * fontWidth;
        if(textWrap && (cursorX > (LCDWIDTH - fontSize * fontWidth)))
		{
            cursorY += fontSize * fontHeight;
            cursorX = 0;
        }
    }
#if ARDUINO >= 100
    return 1;
#endif
}

void Display::setFont(const uint8_t *f)
{
	font = (uint8_t*)f;
	fontWidth = pgm_read_byte(font) + 1;
	fontHeight = pgm_read_byte(font+1) + 1;
	font += 2; //offset the pointer to star after the width and height bytes
}

void Display::drawChar(int8_t x, int8_t y, unsigned char c, uint8_t size)
{
	int8_t tempcolor = color;

    if((x >= LCDWIDTH) || // Clip right
            (y >= LCDHEIGHT) || // Clip bottom
            ((x + (fontWidth-1) * size - 1) < 0) || // Clip left
            ((y + fontHeight * size - 1) < 0)) // Clip top
	{
        return;
	}

    for(int8_t i = 0; i < fontWidth; i++)
	{
        uint8_t line;
		
        if(i == (fontWidth-1))
		{
            line = 0x0;
		}
        else
		{
            line = pgm_read_byte(font + (c * (fontWidth-1)) + i);
		}
			
        for(int8_t j = 0; j < fontHeight; j++)
		{
            if(line & 0x1)
			{
                if(size == 1) // default size
				{
                    drawPixel(x + i, y + j);
				}
                else
				{ // big size
                    fillRect(x + (i * size), y + (j * size), size, size);
                }
            }
			else if(bgcolor != color)
			{
				color = bgcolor;
                if(size == 1) // default size
				{
                    drawPixel(x + i, y + j);
				}
                else
				{ // big size
                    fillRect(x + i*size, y + j*size, size, size);
                }
				color = tempcolor; //restore color to its initial value
            }
            line >>= 1;
        }
    }
	//restore color to what it was previously
	color = tempcolor;
}
