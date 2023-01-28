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

#include <SPI.h>
#include <stdlib.h>
#include "adsCommand.h"
#include "ads129x.h"
#include "SpiDma.h"

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

void arduinoSetup();
void adsSetup();

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
