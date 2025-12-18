#include <BQ25120A.h>
#include <RGBled.h>
#include <Nicla_System.h>
#include <Arduino_BHY2.h>
#include <ArduinoBLE.h>

SensorXYZ gyroXYZ(SENSOR_ID_GYRO);
SensorOrientation eulerXYZ(SENSOR_ID_ORI);

#define SVC_UUID  "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEService gyroService(SVC_UUID);
BLECharacteristic gyroChar(CHAR_UUID, BLERead | BLENotify,
                           sizeof(uint32_t) + 6*sizeof(float)); // 28 bytes

void setup() {

  Serial.begin(115200);
  nicla::begin();
  nicla::enableCharge(120);
  BHY2.begin(NICLA_STANDALONE);
  gyroXYZ.begin(100);
  eulerXYZ.begin(100);

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) {}
  }

  String addr = BLE.address();
  String suffix = addr.substring(addr.length()-5);
  suffix.replace(":", "");
  String devName = "NiclaQuat-" + suffix;

  BLE.setDeviceName(devName.c_str());
  BLE.setLocalName(devName.c_str());
  BLE.setAdvertisedService(gyroService);

  gyroService.addCharacteristic(gyroChar);
  BLE.addService(gyroService);
  BLE.advertise();

  Serial.println("Advertising as");
  Serial.println(devName);
}

void loop() {
  BHY2.update();
  Serial.println("I am here");

  if (BLE.connected()){
    Serial.println ("HAHSDAHSHASH");}
  if (BLE.connected() && gyroChar.subscribed()
      && gyroXYZ.dataAvailable() && eulerXYZ.dataAvailable()) {
  
  Serial.println("I am hthtghthgtgfhere");
    uint32_t ts = millis();
    float gx = gyroXYZ.x();
    float gy = gyroXYZ.y();
    float gz = gyroXYZ.z();
    float eulerx = eulerXYZ.pitch();
    float eulery = eulerXYZ.roll();
    float eulerz = eulerXYZ.heading();

    Serial.print("Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz);
    Serial.print(" | Euler: ");
    Serial.print(eulerx); Serial.print(", ");
    Serial.print(eulery); Serial.print(", ");
    Serial.println(eulerz);

    uint8_t buf[28];
    memcpy(buf +  0, &ts,     4);
    memcpy(buf +  4, &gx,     4);
    memcpy(buf +  8, &gy,     4);
    memcpy(buf + 12, &gz,     4);
    memcpy(buf + 16, &eulerx, 4);
    memcpy(buf + 20, &eulery, 4);
    memcpy(buf + 24, &eulerz, 4);

    gyroChar.writeValue(buf, sizeof(buf));
  }

  BLE.poll();
}
