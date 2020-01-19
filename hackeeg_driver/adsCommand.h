/*
 * adsCommand.h
 *
 * Copyright © 2013-2020 Starcat LLC / Adam Feuer <adam@starcat.io>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _ADS_COMMAND_H
#define _ADS_COMMAND_H

#include "Arduino.h"

// Arduino Due
// HackEEG Shield v1.5.0
const int IPIN_PWDN = 33;
const int PIN_CLKSEL = 48;
const int IPIN_RESET = 47;

const int PIN_START = 59;

// for multiple boards
#define MAX_BOARDS 4
#define INITIAL_BOARD 0

extern uint8_t current_board;
extern uint8_t cs_pins[MAX_BOARDS];
extern uint8_t drdy_pins[MAX_BOARDS];

const int PIN_DOUT = 11;  // SPI out
const int PIN_DIN = 12;   // SPI in
const int PIN_SCLK = 13;  // SPI clock

void csLow();
void csHigh();
void adcWreg(int reg, int val);
void adcSendCommand(int cmd);
void adcSendCommandLeaveCsActive(int cmd);
int adcRreg(int reg);

#endif // _ADS_COMMAND_H
