// #define _BLE_TRACE_
// #define DEBUG
#include "Nicla_System.h"
#include <Arduino_BHY2.h>
#include <ArduinoBLE.h>

#define BLINK_ON1_MS 100
#define BLINK_ON2_MS 200
#define BLINK_OFF_MS 2000
#ifndef SAMPLE_PERIOD
    #define SAMPLE_PERIOD 10
#endif
#define IDLE_TIMEOUT_MS (30 * 60 * 1000) // 30 minutes

// Define compilation flags (or use CLI and pass as argument)
#define IS_GYR
#define IS_ACC
#define IS_EULER
#define IS_BATTERY
#define ACC_RANGE 4     // +/- 4g (general lifestyle tracking)
#define GYR_RANGE 500  // +/- 500dps (general lifestyle tracking)

#include "sense_hub.h"

// Custom BLE service & characteristic UUIDs
#define BLE_SENSE_UUID(val) ("19b10000-" val "-537e-4f6c-d104768a1214")

BLEService niclaDataService(BLE_SENSE_UUID("0000"));
BLECharacteristic niclaDataCharacteristics(BLE_SENSE_UUID("1001"), BLERead | BLENotify, BUF_SIZE);
BLEByteCharacteristic sleepControlCharacteristic(BLE_SENSE_UUID("1002"), BLEWrite | BLERead);

uint32_t ts;
uint32_t prev_ts = millis();
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
bool is_connected = false;

String devName;
uint32_t last_motion_ts = 0;
bool is_sleeping = false;


#ifdef IS_BATTERY
#define BATTERY_CHECK_PERIOD 60000
BLEService batteryService("180F");
BLEUnsignedCharCharacteristic batteryCharacteristics("2A19", BLERead | BLENotify);
uint32_t prev_battery_ts = 0;
uint8_t oldBatteryLevel;

void updateBatteryLevel()
{
    float batteryLevelRaw = nicla::getBatteryVoltagePercentage();
    if (batteryLevelRaw >= 0)
    {
        uint8_t batteryLevel = (uint8_t) batteryLevelRaw;
        if (batteryLevel != oldBatteryLevel)
        {
#ifdef DEBUG
            Serial.print("Battery Level % is now: ");
            Serial.println(batteryLevel);
#endif
            batteryCharacteristics.writeValue(batteryLevel);  // and update the battery level characteristic
            oldBatteryLevel = batteryLevel;
        }
    }
}

void checkChargingStatus(uint32_t ts)
{
    if (ts - prev_battery_ts >= BATTERY_CHECK_PERIOD)
    {
        prev_battery_ts = ts;
        switch(nicla::getOperatingStatus())
        {
            case OperatingStatus::ChargingComplete:
                nicla::disableCharging();
                break;
            default:
                break;
        }
        updateBatteryLevel();
    }
}
#endif


void sendSample(uint32_t ts)
{
    if((ts - prev_ts) >= SAMPLE_PERIOD)
    {
        prev_ts = ts;
        sequence_id++;
        // main loop
        offset = header_offset;  // write offset into the new packet beyond the reserved locations of mask and payload size 
        dataMask = 0;                                   // mask that identifies contents of the payload

        if(BLE.connected() && niclaDataCharacteristics.subscribed())
        {
            get_sensors_data(buf, &offset, &dataMask);

            if(offset > header_offset)
            {
                memcpy(buf, &dataMask, sizeof(uint8_t));
                memcpy(buf + sizeof(uint8_t), &ts, sizeof(uint32_t));
                memcpy(buf + sizeof(uint8_t) + sizeof(uint32_t), &sequence_id, sizeof(uint32_t));

                niclaDataCharacteristics.writeValue(buf, offset);
            }
        }
    }
}


void blinkLED()
{
    switch (led_state)
    {
        case OFF:
            next_led_color = is_connected ? green : blue;
            blink_period = BLINK_OFF_MS;
            next_led_state = ON1;
            break;
        case ON1:
            next_led_color = off;
            blink_period = BLINK_ON1_MS;
            next_led_state = ON2;
            break;
        case ON2:
            next_led_color = is_connected ? green : blue;
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


void disconnectDataHandler(BLEDevice central) {
#ifdef DEBUG
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
#endif
    is_connected = false;
    for(int i = 0; i < 8; ++i)
    {
        nicla::leds.setColor(red);
        delay(100);
        nicla::leds.setColor(off);
        delay(200);
    }
}


void connectDataHandler(BLEDevice central) {
#ifdef DEBUG
    Serial.print("Connected to central: ");
    Serial.println(central.address());
#endif
    is_connected = true;
    for(int i = 0; i < 6; ++i)
    {
        nicla::leds.setColor(blue);
        delay(100);
        nicla::leds.setColor(off);
        delay(200);
        nicla::leds.setColor(red);
        delay(100);
    }
    nicla::leds.setColor(off);
}


void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif
    nicla::begin();                 // board init
    nicla::leds.begin();
    nicla::leds.setColor(white);
#ifdef IS_BATTERY
    float initialLevel = nicla::getBatteryVoltagePercentage();
    oldBatteryLevel = initialLevel >= 0 ? (uint8_t)initialLevel : 0;
    nicla::setBatteryNTCEnabled(true);
    nicla::configureChargingSafetyTimer(ChargingSafetyTimerOption::ThreeHours);
    nicla::enableCharging(100);
#endif
    BHY2.begin(NICLA_BLE, NICLA_AS_SHIELD);   // start Bosch Sensor-Fusion.

    setup_sensors();
#ifdef DEBUG
    Serial.println("Configured sensors");
#endif

    // BLE setup. Block and turn red if failed
    if (!BLE.begin())
    {
        #ifdef DEBUG
        Serial.println("BLE init failed");
        #endif
        nicla::leds.setColor(red);
        while (1);
    }

    // build a unique name from the last 2 bytes of the BLE address:
    String addr = BLE.address();                        // e.g. "AB:CD:EF:12:34:56"
    String suffix = addr.substring(addr.length()-5);    // "34:56"
    suffix.replace(":", "");                            // "3456"
    devName = "NiclaExo-" + suffix;              // e.g. "NiclaExo-3456"
    
    // add & advertise the service
    BLE.setLocalName(devName.c_str());
    BLE.setDeviceName(devName.c_str());
    BLE.setAdvertisedService(niclaDataService);

    niclaDataService.addCharacteristic(niclaDataCharacteristics);
    niclaDataService.addCharacteristic(sleepControlCharacteristic);
    sleepControlCharacteristic.writeValue(0x00);
    BLE.addService(niclaDataService);
    BLE.setEventHandler(BLEDisconnected, disconnectDataHandler);
    BLE.setEventHandler(BLEConnected, connectDataHandler);

#ifdef IS_BATTERY
    batteryService.addCharacteristic(batteryCharacteristics);
    BLE.addService(batteryService);
    batteryCharacteristics.writeValue(oldBatteryLevel);
#endif

#ifdef DEBUG
    Serial.println("Configured BLE");
#endif

    BLE.advertise();

#ifdef DEBUG
    Serial.println("BLE Advertising as");
    Serial.println(devName);
#endif
}


void loop()
{
    if (sleepControlCharacteristic.written())
    {
        if (sleepControlCharacteristic.value() == 0x01)
        {
#ifdef DEBUG
            Serial.println("Entering Ship Mode (Shutdown)...");
#endif
            delay(100); // Allow time for BLE to acknowledge
            nicla::enterShipMode();
        }
    }

    // updates sensor fusion algorithm
    BHY2.update();
    
    // prepare the packet with all the available data for I2C to fetch
    ts = millis();

    // blinks once per second during normal operation
    blinkLED();

    // samples the buffer bewtween IMU sensor and the MCU and pushes it through BLE
    sendSample(ts);

#ifdef IS_BATTERY
    checkChargingStatus(ts);
#endif
}
