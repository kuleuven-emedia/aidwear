#include "Nicla_System.h"
#include <Arduino_BHY2.h>
#include <ArduinoBLE.h>

#define CLK 64000000
#ifndef SAMPLE_RATE
    #define SAMPLE_RATE 10
#endif
// Define compilation flags for Arduino IDE (or use CLI and pass as argument)
// #define IS_GYR
// #define IS_EULER
// #define DEBUG

constexpr size_t BUF_SIZE =
    sizeof(uint8_t) +       // mask of modalities contained in the packet
    sizeof(uint32_t) +      // timestamp from sensor's onboard clock
    sizeof(uint32_t) +      // sequence id of the sample
#ifdef IS_ACC
    3*sizeof(int16_t) +    // X,Y,Z calibrated linear acceleration w.r.t sensor
#else
    0 +
#endif
#ifdef IS_GYR               // X,Y,Z calibrated angular velocity w.r.t. sensor
    3*sizeof(int16_t) +
#else
    0 +
#endif
#ifdef IS_MAG               // X,Y,Z calibrated magnetic field w.r.t. sensor
    3*sizeof(int16_t) +
#else
    0 +
#endif
#ifdef IS_EULER             // X,Y,Z absolute euler angles from the onboard 9-DOF fusion algorithm w.r.t. starting orientation
    3*sizeof(float) +
#else
    0 +
#endif
#ifdef IS_QUAT              // W,X,Y,Z absolute quaternion from the onboard 9-DOF fusion algorithm w.r.t. starting orientation
    4*sizeof(float) +
#else
    0 +
#endif
#ifdef IS_TEMP              // compensated temperature in Celsius
    sizeof(float) +
#else
    0 +
#endif
#ifdef IS_BARO              // atmospheric pressure in hPa
    sizeof(float) +
#else
    0 +
#endif
#ifdef IS_HUM               // relative humidity in %
    sizeof(float);
#else
    0;
#endif
// #ifdef IS_GAS
//     sizeof(float) +
// #else
//     0 +
// #endif
// #ifdef IS_BSEC2
//     sizeof(uint16_t) +  // IAQ for regular (iaq) or stationary (iaq_s) use cases
//     sizeof(float) +     // breath VOC equivalent (ppm)
//     sizeof(uint32_t) +  // CO2 equivalent (ppm) [400,]
//     sizeof(float) +     // compensated temperature (Celsius)
//     sizeof(float) +     // compensated humidity
//     sizeof(uint32_t) +  // compensated gas resistance (Ohm)
//     sizeof(uint8_t);    // accuracy level [0-3]
// #else
//     0;
// #endif

#ifdef IS_ACC
    #define ACC_MASK 0x01
    int16_t accX, accY, accZ;
    SensorXYZ acc(SENSOR_ID_ACC);
#endif
#ifdef IS_GYR
    #define GYR_MASK 0x02
    int16_t gyrX, gyrY, gyrZ;
    SensorXYZ gyr(SENSOR_ID_GYRO);
#endif
#ifdef IS_MAG
    #define MAG_MASK 0x04
    int16_t magX, magY, magZ;
    SensorXYZ mag(SENSOR_ID_MAG);
#endif
#ifdef IS_EULER
    #define EULER_MASK 0x08
    float eulerX, eulerY, eulerZ;
    SensorOrientation euler(SENSOR_ID_ORI);
#endif
#ifdef IS_QUAT
    #define QUAT_MASK 0x10
    float quatW, quatX, quatY, quatZ;
    SensorQuaternion quat(SENSOR_ID_RV);
#endif
#ifdef IS_TEMP
    #define TEMP_MASK 0x20
    float temperatureValue;
    Sensor temperature(SENSOR_ID_TEMP);
#endif
#ifdef IS_BARO
    #define BARO_MASK 0x40
    float pressureValue;
    Sensor pressure(SENSOR_ID_BARO);
#endif
#ifdef IS_HUM
    #define HUM_MASK 0x80
    float humidityValue;
    Sensor humidity(SENSOR_ID_HUM);
#endif
// #ifdef IS_GAS
//     Sensor gas(SENSOR_ID_GAS);
// #endif
// #ifdef IS_BSEC2
//     SensorBSEC2Collector bsec2Collector(SENSOR_ID_BSEC2_COLLECTOR);
// #endif

// Custom BLE service & characteristic UUIDs
#define BLE_SENSE_UUID(val) ("19b10000-" val "-537e-4f6c-d104768a1214")

BLEService niclaService(BLE_SENSE_UUID("0000"));
BLECharacteristic niclaCharacteristics(BLE_SENSE_UUID("1001"), BLERead | BLENotify, BUF_SIZE);


void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
    BHY2.debug(Serial);
#endif
    nicla::begin();                 // board init
    nicla::enableCharge(120);       // enable battery charging through the board
    nicla::leds.begin();
    nicla::leds.setColor(yellow);
    BHY2.begin(NICLA_STANDALONE);   // start Bosch Sensor-Fusion NICLA_AS_SHIELD. TODO: start communication mode in NICLA_BLE

#ifdef IS_ACC
    acc.begin(100);     // 100 Hz Linear acceleration (ds/dt^2)
#endif
#ifdef IS_GYR
    gyr.begin(100);     // 100 Hz Gyroscope (dv/dt)
#endif
#ifdef IS_MAG
    mag.begin(100);     // 100 Hz Magnetic field
#endif
#ifdef IS_EULER
    euler.begin(100);   // 100 Hz Euler angles
#endif
#ifdef IS_QUAT
    quat.begin(100);    // 100 Hz Quaternion
#endif
#ifdef IS_TEMP
    temperature.begin();
#endif
#ifdef IS_BARO
    pressure.begin();
#endif
#ifdef IS_HUM
    humidity.begin();
#endif
// #ifdef IS_GAS
//     float gasValue;
//     gas.begin();
// #endif

    // BLE setup. Block and turn red if failed
    if (!BLE.begin())
    {
        Serial.println("BLE init failed");
        nicla::leds.setColor(red);
        while (1);
    }

    // build a unique name from the last 2 bytes of the BLE address:
    String addr = BLE.address();                        // e.g. "AB:CD:EF:12:34:56"
    String suffix = addr.substring(addr.length()-5);    // "34:56"
    suffix.replace(":", "");                            // "3456"
    String devName = "NiclaQuat-" + suffix;             // e.g. "NiclaQuat-3456"

    // add & advertise the service
    BLE.setLocalName(devName.c_str());
    BLE.setDeviceName(devName.c_str());
    BLE.setAdvertisedService(niclaService);
    niclaService.addCharacteristic(niclaCharacteristics);
    BLE.addService(niclaService);
    BLE.setEventHandler(BLEDisconnected, disconnectHandler);
    BLE.setEventHandler(BLEConnected, connectHandler);

    BLE.advertise();

    Serial.println("BLE Advertising as");
    Serial.println(devName);
}

uint32_t ts;
uint32_t prev_ts = millis();
uint32_t sequence_id = 0;
uint8_t buf[BUF_SIZE];
uint8_t offset;
uint8_t dataMask;
uint32_t blink_counter = 0;
bool toggle = true;

void loop()
{
    // updates sensor fusion algorithm
    BHY2.update();

    // blinks once per second during normal operation
    blinkLED();

    ts = millis();
    if((ts - prev_ts) >= SAMPLE_RATE)
    {
        prev_ts = ts;
        sequence_id++;
        // main loop
        if(BLE.connected() && niclaCharacteristics.subscribed())
        {
            offset = sizeof(uint8_t) + 2*sizeof(uint32_t);  // write offset into the new packet beyond the reserved locations of mask and payload size 
            dataMask = 0;                                   // mask that identifies contents of the payload

        #ifdef IS_ACC
            accX = acc.x();
            accY = acc.y();
            accZ = acc.z();

            memcpy(buf + offset, &accX, sizeof(int16_t));
            offset += sizeof(int16_t);
            memcpy(buf + offset, &accY, sizeof(int16_t));
            offset += sizeof(int16_t);
            memcpy(buf + offset, &accZ, sizeof(int16_t));
            offset += sizeof(int16_t);

            dataMask |= ACC_MASK;
        #endif
        #ifdef IS_GYR
            gyrX = gyr.x();
            gyrY = gyr.y();
            gyrZ = gyr.z();

            memcpy(buf + offset, &gyrX, sizeof(int16_t));
            offset += sizeof(int16_t);
            memcpy(buf + offset, &gyrY, sizeof(int16_t));
            offset += sizeof(int16_t);
            memcpy(buf + offset, &gyrZ, sizeof(int16_t));
            offset += sizeof(int16_t);

            dataMask |= GYR_MASK;
        #endif
        #ifdef IS_MAG
            magX = mag.x();
            magY = mag.y();
            magZ = mag.z();

            memcpy(buf + offset, &magX, sizeof(int16_t));
            offset += sizeof(int16_t);
            memcpy(buf + offset, &magY, sizeof(int16_t));
            offset += sizeof(int16_t);
            memcpy(buf + offset, &magZ, sizeof(int16_t));
            offset += sizeof(int16_t);

            dataMask |= MAG_MASK;
        #endif
        #ifdef IS_EULER
            eulerX = euler.roll();
            eulerY = euler.pitch();
            eulerZ = euler.heading();

            memcpy(buf + offset, &eulerX, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &eulerY, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &eulerZ, sizeof(float));
            offset += sizeof(float);

            dataMask |= EULER_MASK;
        #endif
        #ifdef IS_QUAT
            quatW = quat.w();
            quatX = quat.x();
            quatY = quat.y();
            quatZ = quat.z();
            
            memcpy(buf + offset, &quatW, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &quatX, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &quatY, sizeof(float));
            offset += sizeof(float);
            memcpy(buf + offset, &quatZ, sizeof(float));
            offset += sizeof(float);

            dataMask |= QUAT_MASK;
        #endif
        #ifdef IS_TEMP
            temperatureValue = temperature.value()

            memcpy(buf + offset, &temperatureValue, sizeof(float));
            offset += sizeof(float);

            dataMask |= TEMP_MASK;
        #endif
        #ifdef IS_BARO
            pressureValue = pressure.value()

            memcpy(buf + offset, &pressureValue, sizeof(float));
            offset += sizeof(float);

            dataMask |= BARO_MASK;
        #endif
        #ifdef IS_HUM
            humidityValue = temperature.value()

            memcpy(buf + offset, &humidityValue, sizeof(float));
            offset += sizeof(float);

            dataMask |= HUM_MASK;
        #endif
        // #ifdef IS_GAS
        //     if(gas.dataAvailable())
        //     {
        //         gasValue = gas.value()

        //         memcpy(buf + offset, &gasValue, sizeof(float));
        //         offset += sizeof(float);
        //     }
        // #endif

            if(offset > (sizeof(uint8_t) + 2*sizeof(uint32_t)))
            {
                memcpy(buf, &dataMask, sizeof(uint8_t));
                memcpy(buf + sizeof(uint8_t), &ts, sizeof(uint32_t));
                memcpy(buf + sizeof(uint8_t) + sizeof(uint32_t), &sequence_id, sizeof(uint32_t));

                niclaCharacteristics.writeValue(buf, offset);
            }
        }
    }
}

void disconnectHandler(BLEDevice central) {
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    for(int i = 0; i < 20; ++i)
    {
        delay(100);
        nicla::leds.setColor(red);
        delay(100);
        nicla::leds.setColor(off);
    }
}

void connectHandler(BLEDevice central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    for(int i = 0; i < 20; ++i)
    {
        delay(100);
        nicla::leds.setColor(blue);
        delay(100);
        nicla::leds.setColor(off);
    }
}

void blinkLED()
{
    blink_counter++;
    if(blink_counter >= CLK)
    {
        nicla::leds.setColor(toggle ? yellow : off);
        toggle = !toggle;
        blink_counter = 0;
    }
}
