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

#ifndef DISPLAY_H
#define	DISPLAY_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>

#include "../settings.c"

//colors
#define WHITE 0
#define BLACK 1
#define INVERT 2
#define GRAY 3

#define OLED_WHITE 0xff
#define OLED_BLACK 0x00
#define OLED_GRAY 0x88

//for extended bitmap function :
#define NOROT 0
#define ROTCCW 1
#define ROT180 2
#define ROTCW 3
#define NOFLIP 0
#define FLIPH 1
#define FLIPV 2
#define FLIPVH 3

#if (DISPLAY_EMULATE_84x48 == 1)

#if (DISPLAY_ROT == NO_ROT)||(DISPLAY_ROT == ROT180) //landscape mode
#define LCDWIDTH 84
#define LCDHEIGHT 48
#else //portrait mode
#define LCDWIDTH 48
#define LCDHEIGHT 84
#endif
#define LCDHEIGHT_NOROT 48
#define LCDWIDTH_NOROT 84

#else

#if (DISPLAY_ROT == NO_ROT)||(DISPLAY_ROT == ROT180) //landscape mode
#define LCDWIDTH 96
#define LCDHEIGHT 64
#else //portrait mode
#define LCDWIDTH 64
#define LCDHEIGHT 96
#endif
#define LCDHEIGHT_NOROT 64
#define LCDWIDTH_NOROT 96

#endif

#define swap(a, b) { int8_t t = a; a = b; b = t; }

// SSD1332 Registers
static const uint8_t CMD_NOP				= 0xE3;
static const uint8_t CMD_DRAWLINE			= 0x21;
static const uint8_t CMD_DRAWRECT			= 0x22;
static const uint8_t CMD_DRAWCOPY			= 0x23;
static const uint8_t CMD_DIMWINDOW			= 0x24;
static const uint8_t CMD_CLRWINDOW			= 0x25;
static const uint8_t CMD_FILL				= 0x26;
static const uint8_t CMD_SCROLL_OFF 		= 0x2E;
static const uint8_t CMD_SCROLL_ON  		= 0x2F;
static const uint8_t CMD_SCROLL_SET  		= 0x2F;
static const uint8_t CMD_STARTLINE			= 0xA1;
static const uint8_t CMD_DISPLAYALLON		= 0xA5;
static const uint8_t CMD_DISPLAYALLOFF		= 0xA6;
static const uint8_t CMD_DINVOF  			= 0x20;
static const uint8_t CMD_PHASEPERIOD		= 0x12;
static const uint8_t CMD_CONTRASTA			= 0x81;
static const uint8_t CMD_CONTRASTB			= 0x82;
static const uint8_t CMD_CONTRASTC			= 0x83;
static const uint8_t CMD_DIMMODESET 		= 0xAB;
static const uint8_t CMD_MASTERCURRENT		= 0x87;
static const uint8_t CMD_SETREMAP			= 0xA0;
static const uint8_t CMD_DISPLAYOFFSET		= 0xA2;
static const uint8_t CMD_SETMULTIPLEX		= 0xA8;
static const uint8_t CMD_SETMASTER			= 0xAD;
static const uint8_t CMD_POWERMODE			= 0xB0;
static const uint8_t CMD_PRECHARGE 			= 0xB1;
static const uint8_t CMD_CLOCKDIV			= 0xB3;
static const uint8_t CMD_GRAYSCALE			= 0xB8;
static const uint8_t CMD_PRECHARGEA 		= 0x8A;
static const uint8_t CMD_PRECHARGEB			= 0x8B;
static const uint8_t CMD_PRECHARGEC 		= 0x8C;
static const uint8_t CMD_LINEARGRAY			= 0xB9;
static const uint8_t CMD_PRECHARGELEVEL 	= 0xBB;
static const uint8_t CMD_VCOMH				= 0xBE;
static const uint8_t CMD_VPACOLORLVL		= 0xBB;
static const uint8_t CMD_VPBCOLORLVL		= 0xBC;
static const uint8_t CMD_VPCCOLORLVL		= 0xBD;
static const uint8_t CMD_NORMALDISPLAY		= 0xA4;
static const uint8_t CMD_CMDLOCK	 		= 0xFD;
static const uint8_t CMD_INVERTDISPLAY		= 0xA7;
static const uint8_t CMD_DISPLAYOFF			= 0xAE;
static const uint8_t CMD_DISPLAYDIM			= 0xAC;
static const uint8_t CMD_DISPLAYON			= 0xAF;
static const uint8_t CMD_SETCOLUMN			= 0x15;
static const uint8_t CMD_SETROW				= 0x75;

extern uint8_t _displayBuffer[];

class Display : public Print {
public:
	void begin(int8_t SCLK, int8_t DIN, int8_t DC, int8_t CS, int8_t RST);

	void command(uint8_t c);
	void data(uint8_t c);
	uint8_t* getBuffer();

	void setContrast(uint8_t val);
	void clear(void);
	void update();

	void setColor(int8_t c);
	void setColor(int8_t c, int8_t bg);
	inline void drawPixel(int8_t x, int8_t y);
	inline uint8_t getPixel(int8_t x, int8_t y);

	void drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1);
	void drawFastVLine(int8_t x, int8_t y, int8_t h);
	void drawFastHLine(int8_t x, int8_t y, int8_t w);
	void drawRect(int8_t x, int8_t y, int8_t w, int8_t h);
	void fillRect(int8_t x, int8_t y, int8_t w, int8_t h);
	void fillScreen(uint8_t color);

	void drawCircle(int8_t x0, int8_t y0, int8_t r);
	void drawCircleHelper(int8_t x0, int8_t y0, int8_t r, uint8_t cornername);
	void fillCircle(int8_t x0, int8_t y0, int8_t r);
	void fillCircleHelper(int8_t x0, int8_t y0, int8_t r, uint8_t cornername, int8_t delta);

	void drawTriangle(int8_t x0, int8_t y0, int8_t x1, int8_t y1, int8_t x2, int8_t y2);
	void fillTriangle(int8_t x0, int8_t y0, int8_t x1, int8_t y1, int8_t x2, int8_t y2);
	void drawRoundRect(int8_t x0, int8_t y0, int8_t w, int8_t h, int8_t radius);
	void fillRoundRect(int8_t x0, int8_t y0, int8_t w, int8_t h, int8_t radius);
	
	void drawBitmap(int8_t x, int8_t y, const uint8_t *bitmap);
	void drawBitmap(int8_t x, int8_t y, const uint8_t *bitmap, uint8_t rotation, uint8_t flip);
	boolean getBitmapPixel(const uint8_t* bitmap, uint8_t x, uint8_t y);
	
	void setFont(const uint8_t* f);
	uint8_t fontWidth, fontHeight;
	void drawChar(int8_t x, int8_t y, unsigned char c, uint8_t size);

	virtual size_t write(uint8_t);
	
	boolean persistence; //disable clean() at each frame if true
	boolean textWrap; // If set, 'wrap' text at right edge of 
	uint8_t fontSize;
	int8_t cursorX, cursorY;
	byte contrast;
	byte frameCount;

private:
	int8_t sclk, din, dc, cs, rst;
	volatile uint8_t *mosiport, *clkport, *csport, *dcport;
	uint8_t mosipinmask, clkpinmask, cspinmask, dcpinmask;
	
	uint8_t *font;
	uint8_t color, bgcolor;
};

inline uint8_t* Display::getBuffer(){
	return _displayBuffer;
}

inline void Display::drawPixel(int8_t x, int8_t y) {
	if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
	return;
	
	byte c = color;
	if(color == INVERT){
	 c = !getPixel(x, y);
	} else if(color == GRAY){
		if(((frameCount & 0x01) ^ ((x & 0x01) ^ (y & 0x01)) == 0)){ //alternative checkers pattern
			c = WHITE;
		} else {
			c= BLACK;
		}
	}
	
	if(c == WHITE){ //white
#if DISPLAY_ROT == NOROT
		_displayBuffer[x + (y / 8) * LCDWIDTH_NOROT] &= ~_BV(y % 8);
#elif DISPLAY_ROT == ROTCCW
		_displayBuffer[LCDHEIGHT - y - 1 + (x / 8) * LCDWIDTH_NOROT] &= ~_BV(x % 8);
#elif DISPLAY_ROT == ROT180
		_displayBuffer[LCDWIDTH - x - 1 + ((LCDHEIGHT - y - 1) / 8) * LCDWIDTH_NOROT] &= ~_BV((LCDHEIGHT - y - 1) % 8);
#elif DISPLAY_ROT == ROTCW
		_displayBuffer[y + ((LCDWIDTH - x - 1) / 8) * LCDWIDTH_NOROT] &= ~_BV((LCDWIDTH - x - 1) % 8);
#endif
		return;
	}
	else { //black
#if DISPLAY_ROT == NOROT
		_displayBuffer[x + (y / 8) * LCDWIDTH_NOROT] |= _BV(y % 8);
#elif DISPLAY_ROT == ROTCCW
		_displayBuffer[LCDHEIGHT - y - 1 + (x / 8) * LCDWIDTH_NOROT] |= _BV(x % 8);
#elif DISPLAY_ROT == ROT180
		_displayBuffer[LCDWIDTH - x - 1 + ((LCDHEIGHT - y - 1) / 8) * LCDWIDTH_NOROT] |= _BV((LCDHEIGHT - y - 1) % 8);
#elif DISPLAY_ROT == ROTCW
		_displayBuffer[y + ((LCDWIDTH - x - 1) / 8) * LCDWIDTH_NOROT] |= _BV((LCDWIDTH - x -1) % 8);
#endif
		return;
	}
}

inline uint8_t Display::getPixel(int8_t x, int8_t y) {
	if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
	return 0;

	return (_displayBuffer[x + (y / 8) * LCDWIDTH] >> (y % 8)) & 0x1;
}


#endif	/* DISPLAY_H */

