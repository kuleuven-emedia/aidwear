#include "Nicla_System.h"
#include <Arduino_BHY2.h>
#include <Wire.h>

#define I2C_ADDRESS 0x12

#define BLINK_ON1_MS 100
#define BLINK_ON2_MS 200
#define BLINK_OFF_MS 2000
#ifndef SAMPLE_PERIOD
    #define SAMPLE_PERIOD 10
#endif

// Define compilation flags for Arduino IDE (or use CLI and pass as argument)
#define IS_GYR
#define IS_EULER
// #define DEBUG

#include "sense_hub.h"

// I2C commands.
// Command pattern selected for easier spotting on the scope.
const uint8_t CMD_START = 0xF0;
const uint8_t CMD_SEND = 0xAA;
const uint8_t CMD_STOP = 0x0F;
const uint8_t CMD_ACK = 0xCC;
const uint8_t CMD_DEFAULT = 0x00;
const uint8_t CMD_DEFAULT_REPLY = 0xAA;

// Store current I2C command.
uint8_t i2c_command = CMD_DEFAULT;
uint32_t ts;
uint32_t prev_ts = millis();
uint32_t prev_toggle_ts = prev_ts;
uint32_t sequence_id = 0;
uint8_t buf[BUF_SIZE];
uint8_t offset;
uint8_t header_offset = sizeof(uint8_t) + 2*sizeof(uint32_t);
uint8_t dataMask;
bool is_produce = false;

uint32_t prev_toggle_ts = prev_ts;
uint32_t blink_period = BLINK_OFF_MS;
enum LedState {
    ON1,
    ON2,
    ON3,
    OFF
};
enum LedState led_state = OFF;
enum LedState next_led_state = ON1;
RGBColors next_led_color = off;

void blinkLED()
{
    switch (led_state)
    {
        case OFF:
            next_led_color = is_produce ? green : blue;
            blink_period = BLINK_OFF_MS;
            next_led_state = ON1;
            break;
        case ON1:
            next_led_color = off;
            blink_period = BLINK_ON1_MS;
            next_led_state = ON2;
            break;
        case ON2:
            next_led_color = is_produce ? green : blue;
            blink_period = BLINK_ON2_MS;
            next_led_state = ON3;
            break;
        case ON3:
            next_led_color = off;
            blink_period = BLINK_ON1_MS;
            next_led_state = OFF;
            break;
    }

    if((ts - prev_toggle_ts) > blink_period)
    {
        nicla::leds.setColor(next_led_color);
        prev_toggle_ts = ts;
        led_state = next_led_state;
    }
}

void receiveEvent(int numBytes) {
    if (numBytes == 1)
    {
        i2c_command = Wire.read();
        #ifdef DEBUG
        Serial.println("Received command");
        Serial.println(i2c_command);
        #endif
    }
}

void requestHandler() {
    if (i2c_command == CMD_START)
    {
        is_produce = true;
        Wire.write(CMD_ACK);
    } else if (i2c_command == CMD_SEND)
    {
        Wire.write(buf, offset);
        digitalWrite(P0_19, LOW);
    } else if (i2c_command == CMD_STOP)
    {
        is_produce = false;
        sequence_id = 0;
        Wire.write(CMD_ACK);
    } else
    {
        Wire.write(CMD_DEFAULT_REPLY);
    }

    // Command executed. Reset to default.
    i2c_command = CMD_DEFAULT;
}

void setup()
{
    #ifdef DEBUG
    Serial.begin(115200);
    #endif
    nicla::begin();                 // board init
    nicla::leds.begin();
    nicla::leds.setColor(yellow);
    BHY2.begin(NICLA_I2C, NICLA_VIA_ESLOV);   // start Bosch Sensor-Fusion.
    Wire.setClock(400000);          // set I2C to 400kHz
    Wire.begin(I2C_ADDRESS);

    #ifdef DEBUG
    Serial.println("Configured I2C");
    #endif
    pinMode(P0_19, OUTPUT); // ESLOV free digital IO pin. Used here as interrupt to RPi that a new sample is available.
    digitalWrite(P0_19, LOW);

    setup_sensors();
    #ifdef DEBUG
    Serial.println("Configured sensors");
    #endif

    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestHandler);
    #ifdef DEBUG
    Serial.println("Configured I2C");
    #endif
}

void loop()
{
    // updates sensor fusion algorithm
    BHY2.update();

    // prepare the packet with all the available data for I2C to fetch
    ts = millis();

    // blinks once per second during normal operation
    blinkLED();

    if(is_produce & ((ts - prev_ts) >= SAMPLE_PERIOD))
    {
        prev_ts = ts;
        sequence_id++;
        // main loop
        offset = header_offset;  // write offset into the new packet beyond the reserved locations of mask and payload size 
        dataMask = 0;                                   // mask that identifies contents of the payload

        get_sensors_data(buf, &offset, &dataMask);

        if(offset > header_offset)
        {
            memcpy(buf, &dataMask, sizeof(uint8_t));
            memcpy(buf + sizeof(uint8_t), &ts, sizeof(uint32_t));
            memcpy(buf + sizeof(uint8_t) + sizeof(uint32_t), &sequence_id, sizeof(uint32_t));
            digitalWrite(P0_19, HIGH);
        }
    }
}
