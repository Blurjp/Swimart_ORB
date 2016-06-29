/*
 * SSD1360.h
 *
 *  Created on: Apr 6, 2016
 *      Author: Jianping Huang
 */

#ifndef _SSD1360_H_
#define _SSD1360_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "mraa/i2c.h"
#include "mraa/gpio.h"
#include "services/services_ids.h"


typedef uint8_t byte;



	void sendCommand(byte command);
	void sendData(byte Data);

	//void printChar(char c, byte X=255, byte Y=255);
	void printChar(char c, byte X, byte Y);
	//void printString(const char *String, byte X=255, byte Y=255, byte numChar=255);
	//void printString(const char *String, byte X, byte Y, byte numChar);
	void printString(const char *String, byte X, byte Y);
	//byte printNumber(long n, byte X=255, byte Y=255);
	byte printNumber(long n, byte X, byte Y);
	//byte printNumber(float float_num, byte prec=6, byte Y=255, byte numChar=255);
	byte printFloatNumber(float num, byte prec, byte Y, byte numChar);
	//void printBigNumber(const char *number, byte column=0, byte page=0, byte numChar=255);
	void printBigNumber(const char *number, byte column, byte page, byte numChar);

	void drawBitmap(const byte *bitmaparray, byte X, byte Y, byte width, byte height);

	void init(void);

	void setCursorXY(byte Column, byte Row);
	void clearDisplay();
	//void clearPage(byte page);

	void setNormalDisplay();
	void setInverseDisplay();
	void setPowerOff();
	void setPowerOn();
	void setPageMode();
	void setHorizontalMode();
	void setBrightness(byte Brightness);

	void scrollRight(byte start, byte end, byte speed);
	void scrollLeft(byte start, byte end, byte speed);
	void scrollDiagRight();
	void scrollDiagLeft();
	void setActivateScroll(byte direction, byte startPage, byte endPage, byte scrollSpeed);
	void setDeactivateScroll();
	void displayInfo(float swimSpeed, int lapCounts, int counterTime);
	void DisplayIdle();
	void DisplayActive();

#endif /* QUARK_SSD13066_H_ */
