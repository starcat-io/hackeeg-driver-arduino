/**
 * send and receive commands from TI ADS129x chips.
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

#include "Arduino.h"
#include "adsCommand.h"
#include "ads129x.h"
#include "SpiDma.h"

// for multiple boards
uint8_t current_board = INITIAL_BOARD;
uint8_t cs_pins[MAX_BOARDS] = {23, 52, 10, 4};
uint8_t drdy_pins[MAX_BOARDS] = {24, 25, 26, 27};

void csLow() {
    digitalWrite(cs_pins[current_board], LOW);
}

void csHigh() {
    digitalWrite(cs_pins[current_board], HIGH);
}

void adcSendCommand(int cmd) {
    csLow();
    spiSend(cmd);
    delayMicroseconds(1);
    csHigh();
}

void adcSendCommandLeaveCsActive(int cmd) {
    csLow();
    spiSend(cmd);
}

void adcWreg(int reg, int val) {
    // see pages 40,43 of datasheet -
    csLow();
    spiSend(ADS129x::WREG | reg);
    delayMicroseconds(2);
    spiSend(0);    // number of registers to be read/written – 1
    delayMicroseconds(2);
    spiSend(val);
    delayMicroseconds(1);
    csHigh();
}

int adcRreg(int reg) {
    uint8_t out = 0;
    csLow();
    spiSend(ADS129x::RREG | reg);
    delayMicroseconds(2);
    spiSend(0);    // number of registers to be read/written – 1
    delayMicroseconds(2);
    out = spiRec();
    delayMicroseconds(1);
    csHigh();
    return ((int) out);
}
