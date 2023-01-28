/*
   Driver for TI ADS129x
   for Arduino Due and Arduino Mega2560

   Copyright (c) 2013-2019 by Adam Feuer <adam@adamfeuer.com>

   This library is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library.  If not, see <http://www.gnu.org/licenses/>.

*/

/*
 * minimal blink board light only driver
 * for porting to NuttX
 * 
 */


#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "ads129x.h"
#include <SPI.h>

// SPI clock divider - 1-255, divides 84Mhz system clock 
// 21 = 4 Mhz
// 6 = 14 Mhz
// 5 = 16.8 Mhz
// 4 = 21 Mhz
// ADS1299 needs the SPI clock to be 20Mhz or less

#define SPI_CLOCK_DIVIDER 5

// constants define pins on Arduino 

// Arduino Due
// HackEEG Shield v1.5.0
const int IPIN_PWDN = 33;
const int PIN_CLKSEL = 48;
const int IPIN_RESET = 47;

const int PIN_START = 59;
const int IPIN_DRDY = 24;   // board 0: JP1, pos. 1
//const int IPIN_DRDY = 25; // board 1: JP1, pos. 2
//const int IPIN_DRDY = 26; // board 2: JP1, pos. 3
//const int IPIN_DRDY = 27; // board 3: JP1, pos. 4

const int PIN_CS = 23;   // board 0: JP2, pos. 3
//const int PIN_CS = 52; // board 1: JP2, pos. 4
//const int PIN_CS = 10; // board 2: JP2, pos. 5
//const int PIN_CS = 4;  // board 3: JP2, pos. 6

//const int PIN_DOUT = 11;  //SPI out
//const int PIN_DIN = 12;   //SPI in
//const int PIN_SCLK = 13;  //SPI clock

void adcWreg(int reg, int val);
void adcSendCommand(int cmd);
void adcSendCommandLeaveCsActive(int cmd);
int adcRreg(int reg);
void arduinoSetup();
void adsSetup();

#define BAUD_RATE 2000000     // WiredSerial ignores this and uses the maximum rate
#define WiredSerial SerialUSB // use the Arduino Due's Native USB port

#define SPI_BUFFER_SIZE 200
#define OUTPUT_BUFFER_SIZE 1000

#define TEXT_MODE 0

#define RESPONSE_OK 200
#define RESPONSE_BAD_REQUEST 400
#define UNRECOGNIZED_COMMAND 406
#define RESPONSE_ERROR 500
#define RESPONSE_NOT_IMPLEMENTED 501
#define RESPONSE_NO_ACTIVE_CHANNELS 502

const char *STATUS_TEXT_OK = "Ok";
const char *STATUS_TEXT_BAD_REQUEST = "Bad request";
const char *STATUS_TEXT_ERROR = "Error";
const char *STATUS_TEXT_NOT_IMPLEMENTED = "Not Implemented";
const char *STATUS_TEXT_NO_ACTIVE_CHANNELS = "No Active Channels";

int protocol_mode = TEXT_MODE;
int max_channels = 0;
int num_active_channels = 0;
boolean active_channels[9]; // reports whether channels 1..9 are active
int num_spi_bytes = 0;
int num_timestamped_spi_bytes = 0;
boolean is_rdatac = false;
boolean base64_mode = true;

char hexDigits[] = "0123456789ABCDEF";

// microseconds timestamp
#define TIMESTAMP_SIZE_IN_BYTES 4
union {
    char timestamp_bytes[TIMESTAMP_SIZE_IN_BYTES];
    unsigned long timestamp;
} timestamp_union;

// SPI input buffer
uint8_t spi_bytes[SPI_BUFFER_SIZE];
uint8_t spi_data_available;

const char *hardware_type = "unknown";
const char *board_name = "HackEEG";
const char *maker_name = "Starcat LLC";
const char *driver_version = "v0.3.0-minimal-test-01";


//////////////////////////////////
// SPI adaptation layer


void spiBegin(uint8_t csPin) {
    SPI.begin();
    pinMode(csPin, OUTPUT);
}

void spiInit(uint8_t bitOrder, uint8_t spiMode, uint8_t spiClockDivider) {
    SPI.setBitOrder((BitOrder) bitOrder);  // MSBFIRST or LSBFIRST
    SPI.setDataMode(spiMode);             // SPI_MODE0, SPI_MODE1; SPI_MODE2; SPI_MODE3
    SPI.setClockDivider(spiClockDivider);
}

/** SPI receive a byte */
uint8_t spiRec() {
    noInterrupts();
    return SPI.transfer(0x00);
    interrupts();
}

/** SPI receive multiple bytes */
uint8_t spiRec(uint8_t *buf, size_t len) {
    memset(buf, 0, len);
    noInterrupts();
    SPI.transfer((void *)buf, len);
    interrupts();
    return 0;
}

/** SPI send a byte */
void spiSend(uint8_t b) {
    noInterrupts();
    SPI.transfer(b);
    interrupts();
}

/** SPI send multiple bytes */
void spiSend(const uint8_t *buf, size_t len) {
    noInterrupts();
    SPI.transfer((void *)buf, len);
    interrupts();
}


//////////////////////////////
// ADS1299 send/receive


void adcSendCommand(int cmd) {
    digitalWrite(PIN_CS, LOW);
    spiSend(cmd);
    delayMicroseconds(1);
    digitalWrite(PIN_CS, HIGH);
}

void adcSendCommandLeaveCsActive(int cmd) {
    digitalWrite(PIN_CS, LOW);
    spiSend(cmd);
}

void adcWreg(int reg, int val) {
    //see pages 40,43 of datasheet -
    digitalWrite(PIN_CS, LOW);
    spiSend(ADS129x::WREG | reg);
    delayMicroseconds(2);
    spiSend(0);    // number of registers to be read/written – 1
    delayMicroseconds(2);
    spiSend(val);
    delayMicroseconds(1);
    digitalWrite(PIN_CS, HIGH);
}

int adcRreg(int reg) {
    uint8_t out = 0;
    digitalWrite(PIN_CS, LOW);
    spiSend(ADS129x::RREG | reg);
    delayMicroseconds(2);
    spiSend(0);    // number of registers to be read/written – 1
    delayMicroseconds(2);
    out = spiRec();
    delayMicroseconds(1);
    digitalWrite(PIN_CS, HIGH);
    return ((int) out);
}

///////////////////////////
// main part of program

void setup() {
    WiredSerial.begin(BAUD_RATE);
    pinMode(PIN_LED, OUTPUT);     // Configure the onboard LED for output
    digitalWrite(PIN_LED, LOW);   // default to LED off

    protocol_mode = TEXT_MODE;
    arduinoSetup();
    adsSetup();
    WiredSerial.println("Ready");
}

void loop() {
    blinkBoardLed();
}


void blinkBoardLed() {
    int numberOfBlinks = 10;
    for (int i=0; i < numberOfBlinks; i++) {
      int state = adcRreg(ADS129x::GPIO);
      state = state & 0xF7;
      state = state | 0x80;
      adcWreg(ADS129x::GPIO, state);
      delay(200);
      state = adcRreg(ADS129x::GPIO);
      state = state & 0x77;
      adcWreg(ADS129x::GPIO, state);
      delay(200);
    }
}


void drdy_interrupt() {
    spi_data_available = 1;
}


void adsSetup() { //default settings for ADS1298 and compatible chips
    using namespace ADS129x;
    // Send SDATAC Command (Stop Read Data Continuously mode)
    spi_data_available = 0;
    attachInterrupt(digitalPinToInterrupt(IPIN_DRDY), drdy_interrupt, FALLING);
    adcSendCommand(SDATAC);
    delay(1000); //pause to provide ads129n enough time to boot up...
    // delayMicroseconds(2);
    delay(100);
    int val = adcRreg(ID);
    switch (val & B00011111) {
        case B10000:
            hardware_type = "ADS1294";
            max_channels = 4;
            break;
        case B10001:
            hardware_type = "ADS1296";
            max_channels = 6;
            break;
        case B10010:
            hardware_type = "ADS1298";
            max_channels = 8;
            break;
        case B11110:
            hardware_type = "ADS1299";
            max_channels = 8;
            break;
        case B11100:
            hardware_type = "ADS1299-4";
            max_channels = 4;
            break;
        case B11101:
            hardware_type = "ADS1299-6";
            max_channels = 6;
            break;
        default:
            max_channels = 0;
    }
    num_spi_bytes = (3 * (max_channels + 1)); //24-bits header plus 24-bits per channel
    if (max_channels == 0) { //error mode
        while (1) {
            digitalWrite(PIN_LED, HIGH);
            delay(500);
            digitalWrite(PIN_LED, LOW);
            delay(500);
        }
    } //error mode

    // All GPIO set to output 0x0000: (floating CMOS inputs can flicker on and off, creating noise)
    adcWreg(GPIO, 0);
    adcWreg(CONFIG3,PD_REFBUF | CONFIG3_const);
    digitalWrite(PIN_START, HIGH);
}

void arduinoSetup() {
    pinMode(PIN_LED, OUTPUT);
    using namespace ADS129x;
    // prepare pins to be outputs or inputs
    //pinMode(PIN_SCLK, OUTPUT); //optional - SPI library will do this for us
    //pinMode(PIN_DIN, OUTPUT); //optional - SPI library will do this for us
    //pinMode(PIN_DOUT, INPUT); //optional - SPI library will do this for us
    //pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_START, OUTPUT);
    pinMode(IPIN_DRDY, INPUT);
    pinMode(PIN_CLKSEL, OUTPUT);// *optional
    pinMode(IPIN_RESET, OUTPUT);// *optional
    //pinMode(IPIN_PWDN, OUTPUT);// *optional
    digitalWrite(PIN_CLKSEL, HIGH); // internal clock
    //start Serial Peripheral Interface
    spiBegin(PIN_CS);
    spiInit(MSBFIRST, SPI_MODE1, SPI_CLOCK_DIVIDER);
    //Start ADS1298
    delay(500); //wait for the ads129n to be ready - it can take a while to charge caps
    digitalWrite(PIN_CLKSEL, HIGH);// *optional
    delay(10); // wait for oscillator to wake up
    digitalWrite(IPIN_PWDN, HIGH); // *optional - turn off power down mode
    digitalWrite(IPIN_RESET, HIGH);
    delay(1000);
    digitalWrite(IPIN_RESET, LOW);
    delay(1);
    digitalWrite(IPIN_RESET, HIGH);
    delay(1);  // *optional Wait for 18 tCLKs AKA 9 microseconds, we use 1 millisecond
} 
