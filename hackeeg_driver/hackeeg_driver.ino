/*
   Driver for HackEEG TI ADS1299 shield
   for Arduino Due and Arduino Mega2560

   Copyright © 2013-2020 Starcat LLC / Adam Feuer <adam@starcat.io>

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
#include <ArduinoJson.h>
#include "adsCommand.h"
#include "ads129x.h"
#include "SerialCommand.h"
#include "JsonCommand.h"
#include "Base64.h"
#include "SpiDma.h"


#define BAUD_RATE 2000000     // WiredSerial ignores this and uses the maximum rate
#define WiredSerial SerialUSB // use the Arduino Due's Native USB port

#define SPI_BUFFER_SIZE 200
#define OUTPUT_BUFFER_SIZE 1000

#define TEXT_MODE 0
#define JSONLINES_MODE 1
#define MESSAGEPACK_MODE 2

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
int num_spi_bytes = 0;
int num_timestamped_spi_bytes = 0;
boolean is_rdatac = false;
boolean base64_mode = true;
int num_boards = 0; // number of HackEEG boards in stack

char hex_digits[] = "0123456789ABCDEF";

// microseconds timestamp
#define TIMESTAMP_SIZE_IN_BYTES 4
union {
    char timestamp_bytes[TIMESTAMP_SIZE_IN_BYTES];
    unsigned long timestamp;
} timestamp_union;

// sample number counter
#define SAMPLE_NUMBER_SIZE_IN_BYTES 4
union {
    char sample_number_bytes[SAMPLE_NUMBER_SIZE_IN_BYTES];
    unsigned long sample_number = 0;
} sample_number_union;

struct HackEegBoard {
    uint8_t active;
    int max_channels;
    int num_active_channels;
    boolean active_channels[8];         // reports whether channels 1..8 are active
    uint8_t spi_bytes[SPI_BUFFER_SIZE]; // SPI input buffer
    uint8_t spi_data_available;         // is DRDY active?
};

HackEegBoard boards[MAX_BOARDS];
int active_boards = 0;
int total_channels = 0;

// char buffer to send via USB
char output_buffer[OUTPUT_BUFFER_SIZE];

const char *hardware_type = "unknown";
const char *board_name = "HackEEG";
const char *maker_name = "Starcat LLC";
const char *driver_version = "v0.4.0";

const char *json_rdatac_header= "{\"C\":200,\"B\":";
const char *json_rdatac_middle = ",\"D\":\"";
const char *json_rdatac_footer= "\"}";

uint8_t messagepack_rdatac_header[] = { 0x82, 0xa1, 0x43, 0xcc, 0xc8, 0xa1, 0x42};
size_t messagepack_rdatac_header_size = sizeof(messagepack_rdatac_header);
uint8_t messagepack_rdatac_middle[] = { 0xa1, 0x44, 0xd9, 0x30};
size_t messagepack_rdatac_middle_size = sizeof(messagepack_rdatac_header);

SerialCommand serialCommand;
JsonCommand jsonCommand;

void arduinoSetup();
int setupBoard();
void detectActiveChannels();
void unrecognized(const char *);
void unrecognizedJsonLines(const char *);

void nopCommand(unsigned char unused1, unsigned char unused2);
void versionCommand(unsigned char unused1, unsigned char unused2);
void statusCommand(unsigned char unused1, unsigned char unused2);
void serialNumberCommand(unsigned char unused1, unsigned char unused2);
void textCommand(unsigned char unused1, unsigned char unused2);
void jsonlinesCommand(unsigned char unused1, unsigned char unused2);
void messagepackCommand(unsigned char unused1, unsigned char unused2);
void ledOnCommand(unsigned char unused1, unsigned char unused2);
void ledOffCommand(unsigned char unused1, unsigned char unused2);
void boardLedOffCommand(unsigned char unused1, unsigned char unused2);
void boardLedOnCommand(unsigned char unused1, unsigned char unused2);
void wakeupCommand(unsigned char unused1, unsigned char unused2);
void standbyCommand(unsigned char unused1, unsigned char unused2);
void resetCommand(unsigned char unused1, unsigned char unused2);
void startCommand(unsigned char unused1, unsigned char unused2);
void stopCommand(unsigned char unused1, unsigned char unused2);
void rdatacCommand(unsigned char unused1, unsigned char unused2);
void sdatacCommand(unsigned char unused1, unsigned char unused2);
void rdataCommand(unsigned char unused1, unsigned char unused2);
void base64ModeOnCommand(unsigned char unused1, unsigned char unused2);
void hexModeOnCommand(unsigned char unused1, unsigned char unused2);
void helpCommand(unsigned char unused1, unsigned char unused2);
void readRegisterCommand(unsigned char unused1, unsigned char unused2);
void writeRegisterCommand(unsigned char unused1, unsigned char unused2);
void readRegisterCommandDirect(unsigned char register_number, unsigned char unused1);
void writeRegisterCommandDirect(unsigned char register_number, unsigned char register_value);
void getCurrentBoardCommand(unsigned char unused1, unsigned char unused2);
void setCurrentBoardCommand(unsigned char new_board, unsigned char unused1);
void senseBoardCommand(unsigned char board_number, unsigned char unused);


void setup() {
    WiredSerial.begin(BAUD_RATE);
    pinMode(PIN_LED, OUTPUT);     // Configure the onboard LED for output
    digitalWrite(PIN_LED, LOW);   // default to LED off

    protocol_mode = TEXT_MODE;
    arduinoSetup();

    delay(200);
    for (int i=0; i<MAX_BOARDS; i++) {
        if (senseBoard(i) > 0) {
            current_board = i;
            int result = setupBoard();
            boards[i].active = 1;
            total_channels += boards[i].max_channels;
            active_boards++;
            if (result < 0) {
                rapidBlinkForever(); // error
            }
        }
    }
    current_board = 0;

    // Setup callbacks for SerialCommand commands
    serialCommand.addCommand("nop", nopCommand);                     // No operation (does nothing)
    serialCommand.addCommand("micros", microsCommand);               // Returns number of microseconds since the program began executing
    serialCommand.addCommand("version", versionCommand);             // Echos the driver version number
    serialCommand.addCommand("status", statusCommand);               // Echos the driver status
    serialCommand.addCommand("serialnumber", serialNumberCommand);   // Echos the board serial number (UUID from the onboard 24AA256UID-I/SN I2S EEPROM)
    serialCommand.addCommand("text", textCommand);                   // Sets the communication protocol to text
    serialCommand.addCommand("jsonlines", jsonlinesCommand);         // Sets the communication protocol to JSONLines
    serialCommand.addCommand("messagepack", messagepackCommand);     // Sets the communication protocol to MessagePack
    serialCommand.addCommand("ledon", ledOnCommand);                 // Turns Arduino Due onboard LED on
    serialCommand.addCommand("ledoff", ledOffCommand);               // Turns Arduino Due onboard LED off
    serialCommand.addCommand("boardledoff", boardLedOffCommand);     // Turns HackEEG ADS1299 GPIO4 LED off
    serialCommand.addCommand("boardledon", boardLedOnCommand);       // Turns HackEEG ADS1299 GPIO4 LED on
    serialCommand.addCommand("wakeup", wakeupCommand);               // Send the WAKEUP command
    serialCommand.addCommand("standby", standbyCommand);             // Send the STANDBY command
    serialCommand.addCommand("reset", resetCommand);                 // Reset the ADS1299
    serialCommand.addCommand("start", startCommand);                 // Send START command
    serialCommand.addCommand("stop", stopCommand);                   // Send STOP command
    serialCommand.addCommand("rdatac", rdatacCommand);               // Enter read data continuous mode, clear the ringbuffer, and read new data into the ringbuffer
    serialCommand.addCommand("sdatac", sdatacCommand);               // Stop read data continuous mode; ringbuffer data is still available
    serialCommand.addCommand("rdata", rdataCommand);                 // Read one sample of data from each active channel
    serialCommand.addCommand("rreg", readRegisterCommand);           // Read ADS129x register, argument in hex, print contents in hex
    serialCommand.addCommand("wreg", writeRegisterCommand);          // Write ADS129x register, arguments in hex
    serialCommand.addCommand("base64", base64ModeOnCommand);         // RDATA commands send base64 encoded data - default
    serialCommand.addCommand("hex", hexModeOnCommand);               // RDATA commands send hex encoded data
    serialCommand.addCommand("help", helpCommand);                   // Print list of commands
    serialCommand.setDefaultHandler(unrecognized);                   // Handler for any command that isn't matched

    // Setup callbacks for JsonCommand commands
    jsonCommand.addCommand("nop", nopCommand);                            // No operation (does nothing)
    jsonCommand.addCommand("micros", microsCommand);                      // Returns number of microseconds since the program began executing
    jsonCommand.addCommand("ledon", ledOnCommand);                        // Turns Arduino Due onboard LED on
    jsonCommand.addCommand("ledoff", ledOffCommand);                      // Turns Arduino Due onboard LED off
    jsonCommand.addCommand("boardledon", boardLedOnCommand);              // Turns HackEEG ADS1299 GPIO4 LED on
    jsonCommand.addCommand("boardledoff", boardLedOffCommand);            // Turns HackEEG ADS1299 GPIO4 LED off
    jsonCommand.addCommand("status", statusCommand);                      // Returns the driver status
    jsonCommand.addCommand("reset", resetCommand);                        // Reset the ADS1299
    jsonCommand.addCommand("start", startCommand);                        // Send START command
    jsonCommand.addCommand("stop", stopCommand);                          // Send STOP command
    jsonCommand.addCommand("rdatac", rdatacCommand);                      // Enter read data continuous mode, clear the ringbuffer, and read new data into the ringbuffer
    jsonCommand.addCommand("sdatac", sdatacCommand);                      // Stop read data continuous mode; ringbuffer data is still available
    jsonCommand.addCommand("serialnumber", serialNumberCommand);          // Returns the board serial number (UUID from the onboard 24AA256UID-I/SN I2S EEPROM)
    jsonCommand.addCommand("text", textCommand);                          // Sets the communication protocol to text
    jsonCommand.addCommand("jsonlines", jsonlinesCommand);                // Sets the communication protocol to JSONLines
    jsonCommand.addCommand("messagepack", messagepackCommand);            // Sets the communication protocol to MessagePack
    jsonCommand.addCommand("rreg", readRegisterCommandDirect);            // Read ADS129x register
    jsonCommand.addCommand("wreg", writeRegisterCommandDirect);           // Write ADS129x register
    jsonCommand.addCommand("rdata", rdataCommand);                        // Read one sample of data from each active channel
    jsonCommand.addCommand("setup_board", setupBoardCommand);             // Sets up the current board and puts the ADS1299 into a known state
    jsonCommand.addCommand("get_current_board", getCurrentBoardCommand);  // Gets the current board number that commands will go to (multiboard configurations)
    jsonCommand.addCommand("set_current_board", setCurrentBoardCommand);  // Sets the current board number that commands will go to (for multiboard configurations)
    jsonCommand.addCommand("sense_board", senseBoardCommand);             // Reads the ID register (0) for the ADS1299 for the board (doesn't change current board)
    jsonCommand.setDefaultHandler(unrecognizedJsonLines);                 // Handler for any command that isn't matched

    WiredSerial.println("Ready");
}

void loop() {
    switch (protocol_mode) {
        case TEXT_MODE:
            serialCommand.readSerial();
            break;
        case JSONLINES_MODE:
        case MESSAGEPACK_MODE:
            jsonCommand.readSerial();
            break;
        default:
            // do nothing
            ;
    }
    sendSamples();
}

long hex_to_long(char *digits) {
    using namespace std;
    char *error;
    long n = strtol(digits, &error, 16);
    if (*error != 0) {
        return -1; // error
    } else {
        return n;
    }
}

void output_hex_byte(int value) {
    int clipped = value & 0xff;
    char charValue[3];
    sprintf(charValue, "%02X", clipped);
    WiredSerial.print(charValue);
}

void encode_hex(char *output, char *input, int input_len) {
    register int count = 0;
    for (register int i = 0; i < input_len; i++) {
        register uint8_t low_nybble = input[i] & 0x0f;
        register uint8_t highNybble = input[i] >> 4;
        output[count++] = hex_digits[highNybble];
        output[count++] = hex_digits[low_nybble];
    }
    output[count] = 0;
}

void send_response_ok() {
    send_response(RESPONSE_OK, STATUS_TEXT_OK);
}

void send_response_error() {
    send_response(RESPONSE_ERROR, STATUS_TEXT_ERROR);
}

void send_response(int status_code, const char *status_text) {
    switch (protocol_mode) {
        case TEXT_MODE:
            char response[128];
            sprintf(response, "%d %s", status_code, status_text);
            WiredSerial.println(response);
            break;
        case JSONLINES_MODE:
            jsonCommand.sendJsonLinesResponse(status_code, (char *) status_text);
            break;
        case MESSAGEPACK_MODE:
            // all responses are in JSON Lines, MessagePack mode is only for sending samples
            jsonCommand.sendJsonLinesResponse(status_code, (char *) status_text);
            break;
        default:
            // unknown protocol
            ;
    }
}

void send_jsonlines_data(int status_code, char data, char *status_text) {
    StaticJsonDocument<1024> doc;
    JsonObject root = doc.to<JsonObject>();
    root[STATUS_CODE_KEY] = status_code;
    root[STATUS_TEXT_KEY] = status_text;
    root[DATA_KEY] = data;
    serializeJson(doc, WiredSerial);
    WiredSerial.println();
    doc.clear();
}

void versionCommand(unsigned char unused1, unsigned char unused2) {
    send_response(RESPONSE_OK, driver_version);
}

void statusCommand(unsigned char unused1, unsigned char unused2) {
    detectActiveChannels();
    int active_channels = 0;
    for (int i=0; i<MAX_BOARDS; i++) {
        if (boards[i].active > 0) {
            active_channels += boards[i].num_active_channels;
        }
    }
    if (protocol_mode == TEXT_MODE) {
        WiredSerial.println("200 Ok");
        WiredSerial.print("Driver version: ");
        WiredSerial.println(driver_version);
        WiredSerial.print("Board name: ");
        WiredSerial.println(board_name);
        WiredSerial.print("Board maker: ");
        WiredSerial.println(maker_name);
        WiredSerial.print("Hardware type: ");
        WiredSerial.println(hardware_type);
        WiredSerial.print("Active boards: ");
        WiredSerial.println(active_boards);
        WiredSerial.print("Total channels: ");
        WiredSerial.println(total_channels);
        WiredSerial.print("Number of active channels: ");
        WiredSerial.println(active_channels);
        WiredSerial.println();
        return;
    }
    StaticJsonDocument<1024> doc;
    JsonObject root = doc.to<JsonObject>();
    root[STATUS_CODE_KEY] = STATUS_OK;
    root[STATUS_TEXT_KEY] = STATUS_TEXT_OK;
    JsonObject status_info = root.createNestedObject(DATA_KEY);
    status_info["driver_version"] = driver_version;
    status_info["board_name"] = board_name;
    status_info["maker_name"] = maker_name;
    status_info["hardware_type"] = hardware_type;
    status_info["total_channels"] = total_channels;
    status_info["active_channels"] = active_channels;
    switch (protocol_mode) {
        case JSONLINES_MODE:
        case MESSAGEPACK_MODE:
            jsonCommand.sendJsonLinesDocResponse(doc);
            break;
        default:
            // unknown protocol
            ;
    }
}

void nopCommand(unsigned char unused1, unsigned char unused2) {
    send_response_ok();
}

void microsCommand(unsigned char unused1, unsigned char unused2) {
    unsigned long microseconds = micros();
    if (protocol_mode == TEXT_MODE) {
        send_response_ok();
        WiredSerial.println(microseconds);
        return;
    }
    StaticJsonDocument<1024> doc;
    JsonObject root = doc.to<JsonObject>();
    root[STATUS_CODE_KEY] = STATUS_OK;
    root[STATUS_TEXT_KEY] = STATUS_TEXT_OK;
    root[DATA_KEY] = microseconds;
    switch (protocol_mode) {
        case JSONLINES_MODE:
        case MESSAGEPACK_MODE:
            jsonCommand.sendJsonLinesDocResponse(doc);
            break;
        default:
            // unknown protocol
            ;
    }
}

void serialNumberCommand(unsigned char unused1, unsigned char unused2) {
    send_response(RESPONSE_NOT_IMPLEMENTED, STATUS_TEXT_NOT_IMPLEMENTED);
}

void textCommand(unsigned char unused1, unsigned char unused2) {
    protocol_mode = TEXT_MODE;
    send_response_ok();
}

void jsonlinesCommand(unsigned char unused1, unsigned char unused2) {
    protocol_mode = JSONLINES_MODE;
    send_response_ok();
}

void messagepackCommand(unsigned char unused1, unsigned char unused2) {
    protocol_mode = MESSAGEPACK_MODE;
    send_response_ok();
}

void ledOnCommand(unsigned char unused1, unsigned char unused2) {
    digitalWrite(PIN_LED, HIGH);
    send_response_ok();
}

void ledOffCommand(unsigned char unused1, unsigned char unused2) {
    digitalWrite(PIN_LED, LOW);
    send_response_ok();
}

void boardLedOn() {
    int state = adcRreg(ADS129x::GPIO);
    state = state & 0xF7;
    state = state | 0x80;
    adcWreg(ADS129x::GPIO, state);
}

void boardLedOff() {
    int state = adcRreg(ADS129x::GPIO);
    state = state & 0x77;
    adcWreg(ADS129x::GPIO, state);
}

void blinkBoardLed() {
    boardLedOn();
    delay(200);
    boardLedOff();
}

void boardLedOnCommand(unsigned char unused1, unsigned char unused2) {
    boardLedOn();
    send_response_ok();
}

void boardLedOffCommand(unsigned char unused1, unsigned char unused2) {
    boardLedOff();
    send_response_ok();
}

void base64ModeOnCommand(unsigned char unused1, unsigned char unused2) {
    base64_mode = true;
    send_response(RESPONSE_OK, "Base64 mode on - rdata command will respond with base64 encoded data.");
}

void hexModeOnCommand(unsigned char unused1, unsigned char unused2) {
    base64_mode = false;
    send_response(RESPONSE_OK, "Hex mode on - rdata command will respond with hex encoded data");
}

void helpCommand(unsigned char unused1, unsigned char unused2) {
    if (protocol_mode == JSONLINES_MODE ||  protocol_mode == MESSAGEPACK_MODE) {
        send_response(RESPONSE_OK, "Help not available in JSON Lines or MessagePack modes.");
    } else {
        WiredSerial.println("200 Ok");
        WiredSerial.println("Available commands: ");
        serialCommand.printCommands();
        WiredSerial.println();
    }
}

void readRegisterCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    char *arg1;
    arg1 = serialCommand.next();
    if (arg1 != NULL) {
        long registerNumber = hex_to_long(arg1);
        if (registerNumber >= 0) {
            int result = adcRreg(registerNumber);
            WiredSerial.print("200 Ok");
            WiredSerial.print(" (Read Register ");
            output_hex_byte(registerNumber);
            WiredSerial.print(") ");
            WiredSerial.println();
            output_hex_byte(result);
            WiredSerial.println();
        } else {
            WiredSerial.println("402 Error: expected hexidecimal digits.");
        }
    } else {
        WiredSerial.println("403 Error: register argument missing.");
    }
    WiredSerial.println();
}

void readRegisterCommandDirect(unsigned char register_number, unsigned char unused1) {
    using namespace ADS129x;
    if (register_number >= 0 and register_number <= 255) {
        unsigned char result = adcRreg(register_number);
        StaticJsonDocument<1024> doc;
        JsonObject root = doc.to<JsonObject>();
        root[STATUS_CODE_KEY] = STATUS_OK;
        root[STATUS_TEXT_KEY] = STATUS_TEXT_OK;
        root[DATA_KEY] = result;
        jsonCommand.sendJsonLinesDocResponse(doc);
    } else {
        send_response_error();
    }
}

void writeRegisterCommand(unsigned char unused1, unsigned char unused2) {
    char *arg1, *arg2;
    arg1 = serialCommand.next();
    arg2 = serialCommand.next();
    if (arg1 != NULL) {
        if (arg2 != NULL) {
            long registerNumber = hex_to_long(arg1);
            long registerValue = hex_to_long(arg2);
            if (registerNumber >= 0 && registerValue >= 0) {
                adcWreg(registerNumber, registerValue);
                WiredSerial.print("200 Ok");
                WiredSerial.print(" (Write Register ");
                output_hex_byte(registerNumber);
                WiredSerial.print(" ");
                output_hex_byte(registerValue);
                WiredSerial.print(") ");
                WiredSerial.println();
            } else {
                WiredSerial.println("402 Error: expected hexidecimal digits.");
            }
        } else {
            WiredSerial.println("404 Error: value argument missing.");
        }
    } else {
        WiredSerial.println("403 Error: register argument missing.");
    }
    WiredSerial.println();
}


void writeRegisterCommandDirect(unsigned char register_number, unsigned char register_value) {
    if (register_number >= 0 && register_value >= 0) {
        adcWreg(register_number, register_value);
        send_response_ok();
    } else {
        send_response_error();
    }
}

void getCurrentBoardCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    StaticJsonDocument<1024> doc;
    JsonObject root = doc.to<JsonObject>();
    root[STATUS_CODE_KEY] = STATUS_OK;
    root[STATUS_TEXT_KEY] = STATUS_TEXT_OK;
    root[DATA_KEY] = (int) current_board;
    jsonCommand.sendJsonLinesDocResponse(doc);
}

void setCurrentBoardCommand(unsigned char new_board, unsigned char unused1) {
    if ((new_board >= 0) && (new_board < MAX_BOARDS)) {
        setCurrentBoard((uint8_t) new_board);
        send_response_ok();
    } else {
        send_response_error();
    }
}

void setupBoardCommand(unsigned char unused1, unsigned char unused2) {
    int result = setupBoard();
    if (result >= 0) {
        send_response_ok();
    } else {
        send_response_error();
    }
}

// returns the contents of the ADS1299 ID register... nonzero means there is a board there.
int senseBoard(unsigned char board_number) {
    using namespace ADS129x;
    if ((board_number >= 0) && (board_number < MAX_BOARDS)) {
        uint8_t old_board = current_board;
        current_board = (uint8_t) board_number;
        adcSendCommand(SDATAC);
//        delay(1000); //pause to provide ADS1299 enough time to boot up...
        delay(200); //pause to provide ADS1299 enough time to boot up...
        unsigned char result = adcRreg(ID);
        current_board = old_board;
        return result;
    } else {
        return -1;
    }
}

void senseBoardCommand(unsigned char board_number, unsigned char unused) {
    int result = senseBoard(board_number);
    if (result >= 0) {
        StaticJsonDocument<1024> doc;
        JsonObject root = doc.to<JsonObject>();
        root[STATUS_CODE_KEY] = STATUS_OK;
        root[STATUS_TEXT_KEY] = STATUS_TEXT_OK;
        root[DATA_KEY] = result;
        jsonCommand.sendJsonLinesDocResponse(doc);
    } else {
        send_response_error();
    }
}

void wakeupCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    adcSendCommand(WAKEUP);
    send_response_ok();
}

void standbyCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    adcSendCommand(STANDBY);
    send_response_ok();
}

void resetCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    adcSendCommand(RESET);
    int result = setupBoard();
    if (result >= 0) {
        send_response_ok();
    } else {
        send_response_error();
    }
}

void startCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    adcSendCommand(START);
    sample_number_union.sample_number = 0;
    send_response_ok();
}

void stopCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    adcSendCommand(STOP);
    send_response_ok();
}

void rdataCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    while (digitalRead(drdy_pins[current_board]) == HIGH);
    adcSendCommandLeaveCsActive(RDATA);
    if (protocol_mode == TEXT_MODE) {
        send_response_ok();
    }
    sendSample(current_board);
}

void rdatacCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    detectActiveChannels();
    if (total_channels > 0) {
        is_rdatac = true;
        synchronizeDrdyOnAllBoards();
        adcSendCommand(RDATAC);
        send_response_ok();
    } else {
        send_response(RESPONSE_NO_ACTIVE_CHANNELS, STATUS_TEXT_NO_ACTIVE_CHANNELS);
    }
}

void sdatacCommand(unsigned char unused1, unsigned char unused2) {
    using namespace ADS129x;
    is_rdatac = false;
    adcSendCommand(SDATAC);
    using namespace ADS129x;
    send_response_ok();
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
    WiredSerial.println("406 Error: Unrecognized command.");
    WiredSerial.println();
}

// This gets set as the default handler for jsonlines and messagepack, and gets called when no other command matches.
void unrecognizedJsonLines(const char *command) {
    StaticJsonDocument<1024> doc;
    JsonObject root = doc.to<JsonObject>();
    root[STATUS_CODE_KEY] = UNRECOGNIZED_COMMAND;
    root[STATUS_TEXT_KEY] = "Unrecognized command";
    jsonCommand.sendJsonLinesDocResponse(doc);
}

void detectActiveChannels() {  //set device into RDATAC (continous) mode -it will stream data
    if ((is_rdatac) || (total_channels < 1)) return; //we can not read registers when in RDATAC mode
    //Serial.println("Detect active channels: ");
    using namespace ADS129x;
    for (int board = 0; board < MAX_BOARDS; board++) {
        boards[board].num_active_channels = 0;
        for (int i = 1; i <= boards[board].max_channels; i++) {
            delayMicroseconds(1);
            int chSet = adcRreg(board, CHnSET + i);
            boards[board].active_channels[i] = ((chSet & 7) != SHORTED);
            if ((chSet & 7) != SHORTED) boards[board].num_active_channels++;
        }

    }
}

void drdyInterruptBoard0() {
    boards[0].spi_data_available = 1;
}

void drdyInterruptBoard1() {
    boards[1].spi_data_available = 1;
}

void drdyInterruptBoard2() {
    boards[2].spi_data_available = 1;
}

void drdyInterruptBoard3() {
    boards[3].spi_data_available = 1;
}

typedef void (*f)();
f drdyInterruptHandlers[4] = {&drdyInterruptBoard0,
                            &drdyInterruptBoard1,
                            &drdyInterruptBoard2,
                            &drdyInterruptBoard3};


// TODO: unroll loop
inline void sendSamples(void) {
    if (!is_rdatac) return;
    for (uint8_t board = 0; board < MAX_BOARDS; board++) {
        if ((boards[board].spi_data_available) && (boards[board].active > 0)) {
            boards[board].spi_data_available = 0;
            receiveSample(board);
            sendSample(board);
        }
    }
}

inline void receiveSample(uint8_t board) {
    csLow(board);
    delayMicroseconds(10);
    memset(boards[board].spi_bytes, 0, sizeof(boards[board].spi_bytes));
    timestamp_union.timestamp = micros();
    boards[board].spi_bytes[0] = timestamp_union.timestamp_bytes[0];
    boards[board].spi_bytes[1] = timestamp_union.timestamp_bytes[1];
    boards[board].spi_bytes[2] = timestamp_union.timestamp_bytes[2];
    boards[board].spi_bytes[3] = timestamp_union.timestamp_bytes[3];
    boards[board].spi_bytes[4] = sample_number_union.sample_number_bytes[0];
    boards[board].spi_bytes[5] = sample_number_union.sample_number_bytes[1];
    boards[board].spi_bytes[6] = sample_number_union.sample_number_bytes[2];
    boards[board].spi_bytes[7] = sample_number_union.sample_number_bytes[3];

    uint8_t returnCode = spiRec(boards[board].spi_bytes + TIMESTAMP_SIZE_IN_BYTES + SAMPLE_NUMBER_SIZE_IN_BYTES, num_spi_bytes);

    csHigh(board);
    sample_number_union.sample_number++;
}

inline void sendSample(uint8_t board) {
    switch (protocol_mode) {
        case JSONLINES_MODE:
            send_sample_json_fast(board, num_timestamped_spi_bytes);
            break;
        case TEXT_MODE:
            if (base64_mode) {
                base64_encode(output_buffer, (char *) boards[board].spi_bytes, num_timestamped_spi_bytes);
            } else {
                encode_hex(output_buffer, (char *) boards[board].spi_bytes, num_timestamped_spi_bytes);
            }
            WiredSerial.println(output_buffer);
            break;
        case MESSAGEPACK_MODE:
            send_sample_messagepack(num_timestamped_spi_bytes);
            break;
    }
}

inline void send_sample_json_fast(uint8_t board, int num_timestamped_spi_bytes) {
    base64_encode(output_buffer, (char *) boards[board].spi_bytes, num_timestamped_spi_bytes);
    WiredSerial.write(json_rdatac_header);
    WiredSerial.write(hex_digits[board]);
    WiredSerial.write(json_rdatac_middle);
    WiredSerial.write(output_buffer);
    WiredSerial.write(json_rdatac_footer);
    WiredSerial.write("\n");
}

// {"C": 200, "D": "T9C06QIAAADAAAARBhIYMGUu4p8mxHjZV9y6u0Cuxza0ISc=", "B": 0}
inline void send_sample_json(int num_bytes) {
    StaticJsonDocument<1024> doc;
    JsonObject root = doc.to<JsonObject>();
    root[STATUS_CODE_KEY] = STATUS_OK;
    root[STATUS_TEXT_KEY] = STATUS_TEXT_OK;
    root[BOARD_KEY] = current_board;
    JsonArray data = root.createNestedArray(DATA_KEY);
    copyArray(boards[current_board].spi_bytes, num_bytes, data);
    jsonCommand.sendJsonLinesDocResponse(doc);
}

// TODO: test
// {"C": 200, "B": 0, "D": "T9C06QIAAADAAAARBhIYMGUu4p8mxHjZV9y6u0Cuxza0ISc="}
inline void send_sample_messagepack(int num_bytes) {
    WiredSerial.write(messagepack_rdatac_header, messagepack_rdatac_header_size);
    WiredSerial.write((uint8_t) current_board);
    WiredSerial.write(messagepack_rdatac_middle, messagepack_rdatac_middle_size);
    WiredSerial.write((uint8_t) num_bytes);
    WiredSerial.write(boards[current_board].spi_bytes, num_bytes);
}

void setCurrentBoard(uint8_t new_board) {
//    detachInterrupt(digitalPinToInterrupt(drdy_pins[current_board]));
    current_board = (uint8_t) new_board;
}

// initialize ADS1299
int setupBoard() {
    using namespace ADS129x;
    // Send SDATAC Command (Stop Read Data Continuously mode)
    adcSendCommand(SDATAC);
    delay(1000); // pause to provide the ADS1299 enough time to boot up...
    boards[current_board].spi_data_available = 0;
    attachInterrupt(digitalPinToInterrupt(drdy_pins[current_board]),
            drdyInterruptHandlers[current_board],
            FALLING);
    int val = adcRreg(ID);
    switch (val & B00011111) {
        case B10000:
            hardware_type = "ADS1294";
            boards[current_board].max_channels = 4;
            break;
        case B10001:
            hardware_type = "ADS1296";
            boards[current_board].max_channels = 6;
            break;
        case B10010:
            hardware_type = "ADS1298";
            boards[current_board].max_channels = 8;
            break;
        case B11110:
            hardware_type = "ADS1299";
            boards[current_board].max_channels = 8;
            break;
        case B11100:
            hardware_type = "ADS1299-4";
            boards[current_board].max_channels = 4;
            break;
        case B11101:
            hardware_type = "ADS1299-6";
            boards[current_board].max_channels = 6;
            break;
        default:
            boards[current_board].max_channels = 0;
            return -1; // error
    }

    num_spi_bytes = (3 * (boards[current_board].max_channels + 1)); // 24-bits header plus 24-bits per channel
    num_timestamped_spi_bytes = num_spi_bytes + TIMESTAMP_SIZE_IN_BYTES + SAMPLE_NUMBER_SIZE_IN_BYTES;

    // All GPIO set to output 0x0000: (floating CMOS inputs can flicker on and off, creating noise)
    adcWreg(GPIO, 0);
    adcWreg(CONFIG3,PD_REFBUF | CONFIG3_const);
    blinkBoardLed();
    return 0;
}

void synchronizeDrdyOnAllBoards() {
    digitalWrite(PIN_START, HIGH);
    delay(1);
    digitalWrite(PIN_START, LOW);
}

void rapidBlinkForever() {
    // signal we got here
    while (1) {
        digitalWrite(PIN_LED, HIGH);
        delay(100);
        digitalWrite(PIN_LED, LOW);
        delay(100);
    }
}

void arduinoSetup() {
    pinMode(PIN_LED, OUTPUT);
    using namespace ADS129x;


    pinMode(PIN_START, OUTPUT);
    for (int i=0; i< MAX_BOARDS; i++) {
        pinMode(drdy_pins[i], INPUT);
    }

    pinMode(PIN_CLKSEL, OUTPUT); // *optional
    pinMode(IPIN_RESET, OUTPUT); // *optional
    pinMode(IPIN_PWDN, OUTPUT);  // *optional

    spiBegin();
    spiInit(MSBFIRST, SPI_MODE1, SPI_CLOCK_DIVIDER);

    // Start ADS1299
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
