#include "Nicla_System.h"        // must come first
#include <Arduino_BHY2.h>
#include <ArduinoBLE.h>

// Bosch fusion core + rotation-vector sensor

SensorQuaternion quatSensor(SENSOR_ID_RV);
SensorXYZ gyroXYZ(SENSOR_ID_GYRO);
SensorOrientation eulerXYZ(SENSOR_ID_ORI);

// Custom BLE service & characteristic UUIDs
#define Q_SVC_UUID  "19b10000-e8f2-537e-4f6c-d104768a1214"
#define Q_CHAR_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
#define GY_CHAR_UUID "19b10004-e8f2-537e-4f6c-d104768a1214"
#define E_CHAR_UUID "19b10006-e8f2-537e-4f6c-d104768a1214"

BLEService        quatService(Q_SVC_UUID);
BLECharacteristic quatChar(Q_CHAR_UUID, BLERead | BLENotify, sizeof(uint32_t) + 4*sizeof(float));
BLECharacteristic gyroChar(GY_CHAR_UUID, BLENotify, 3 * sizeof(float));                             // Array of 3 floats, dps
BLECharacteristic eulerChar(E_CHAR_UUID, BLENotify, 3 * sizeof(float));                             // Array of 3 floats, dps


void setup()
{
    Serial.begin(115200);
    nicla::begin();                          // board init
    BHY2.begin(NICLA_STANDALONE);            // start Bosch Sensor-Fusion
    quatSensor.begin(100);                   // 100 Hz rotation-vector
    gyroXYZ.begin(100);                      // 100 Hz Gyro 
    eulerXYZ.begin(100);                     // 100 Hz Euler angles

    // BLE setup
    if (!BLE.begin())
    {
        Serial.println("BLE init failed");
        while (1);
    }

    // build a unique name from the last 2 bytes of the BLE address:
    String addr = BLE.address();            // e.g. "AB:CD:EF:12:34:56"
    String suffix = addr.substring(addr.length()-5); // "34:56"
    suffix.replace(":", "");                // "3456"
    String devName = "NiclaQuat-" + suffix; // e.g. "NiclaQuat-3456"

    BLE.setDeviceName(devName.c_str());
    BLE.setLocalName(devName.c_str());
    BLE.setAdvertisedService(quatService);

    // add & advertise the service
    quatService.addCharacteristic(quatChar);
    quatService.addCharacteristic(gyroChar);
    quatService.addCharacteristic(eulerChar);
    BLE.addService(quatService);
    BLE.advertise();

    Serial.println("Advertising as");
    Serial.println(devName);
}


void loop()
{
    BHY2.update();  // empty IMU FIFO
    if (BLE.connected() &&
        quatChar.subscribed() &&
        quatSensor.dataAvailable() &&
        gyroXYZ.dataAvailable() &&
        eulerXYZ.dataAvailable())
    {
      float x, y, z, w, gx, gy, gz, eulerx, eulery, eulerz;
      uint32_t ts = millis(); 
      
      // Quaterinion data buffer 
      uint8_t Qbuf[20];
      x = quatSensor.x();
      y = quatSensor.y();
      z = quatSensor.z();
      w = quatSensor.w();
      
      memcpy(Qbuf +   0, &ts  , 4);
      memcpy(Qbuf +   4, &w   , 4);
      memcpy(Qbuf +   8, &x   , 4);
      memcpy(Qbuf +  12, &y   , 4);
      memcpy(Qbuf +  16, &z   , 4);
    
      quatChar.writeValue(Qbuf, sizeof(Qbuf));
    
      // Gyro Data buffer
      uint8_t Gbuf[12];
      gx = gyroXYZ.x(); 
      gy = gyroXYZ.y();
      gz = gyroXYZ.z();
      
      memcpy(Gbuf +   0, &gx   , 4);
      memcpy(Gbuf +   4, &gy   , 4);
      memcpy(Gbuf +   8, &gz   , 4);

      gyroChar.writeValue(Gbuf, sizeof(Gbuf));

      // Euler Data Buffer
      uint8_t Ebuf[12];
 
      eulerx = eulerXYZ.pitch();
      eulery = eulerXYZ.roll();
      eulerz = eulerXYZ.heading();

      memcpy(Ebuf +   0, &eulerx   , 4);
      memcpy(Ebuf +   4, &eulery   , 4);
      memcpy(Ebuf +   8, &eulerz   , 4);

      eulerChar.writeValue(Ebuf, sizeof(Ebuf));
    }
  BLE.poll();                         // keep stack responsive
}
