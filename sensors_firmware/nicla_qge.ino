#include "Nicla_System.h"
#include <Arduino_BHY2.h>
#include <ArduinoBLE.h>

// Sensor definitions
SensorQuaternion quatSensor(SENSOR_ID_RV);
SensorXYZ gyroSensor(SENSOR_ID_GYRO);
SensorOrientation eulSensor(SENSOR_ID_ORI);

// Service and characteristic UUIDs
#define SERVICE_UUID       "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID_QUAT     "19b10002-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID_GYRO     "19b10003-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID_EUL      "19b10004-e8f2-537e-4f6c-d104768a1214"
#define CHAR_UUID_STATUS   "19b10005-e8f2-537e-4f6c-d104768a1214"

// Sequence counters
uint32_t quatSeq = 0;
uint32_t gyroSeq = 0;
uint32_t eulSeq = 0;

// Buffer sizes - adjust based on available RAM and priorities
#define QUAT_BUFFER_SIZE 25
#define GYRO_BUFFER_SIZE 15
#define EULER_BUFFER_SIZE 10

// Data structures for samples
struct QuatSample {
    uint32_t sequence;
    uint32_t timestamp;
    float w, x, y, z;
    uint8_t checksum;
};

struct GyroSample {
    uint32_t sequence;
    uint32_t timestamp;
    float x, y, z;
    uint8_t checksum;
};

struct EulerSample {
    uint32_t sequence;
    uint32_t timestamp;
    float heading, pitch, roll;
    uint8_t checksum;
};

// Circular buffers
QuatSample quatBuffer[QUAT_BUFFER_SIZE];
GyroSample gyroBuffer[GYRO_BUFFER_SIZE];
EulerSample eulBuffer[EULER_BUFFER_SIZE];

// Buffer indices and state
int quatHead = 0, quatTail = 0;
int gyroHead = 0, gyroTail = 0;
int eulHead = 0, eulTail = 0;
bool quatBufferFull = false;
bool gyroBufferFull = false;
bool eulBufferFull = false;

// BLE service and characteristics
BLEService bleService(SERVICE_UUID);
BLECharacteristic quatChar(CHAR_UUID_QUAT, BLERead | BLENotify, sizeof(QuatSample));
BLECharacteristic gyroChar(CHAR_UUID_GYRO, BLERead | BLENotify, sizeof(GyroSample));
BLECharacteristic eulChar(CHAR_UUID_EUL, BLERead | BLENotify, sizeof(EulerSample));
BLECharacteristic statusChar(CHAR_UUID_STATUS, BLERead | BLENotify, 12);

// Connection state tracking
bool wasConnected = false;
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000; // 5 seconds between reconnection attempts
unsigned long lastStatusUpdate = 0;

// Helper function to calculate checksum
uint8_t calculateChecksum(const uint8_t* buffer, size_t size)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < size; i++)
    {
        checksum ^= buffer[i];
    }
    return checksum;
}

// Buffer management functions
void addToQuatBuffer(QuatSample sample)
{
    if (quatBufferFull)
    {
        // Buffer is full, we'll overwrite oldest data
        quatTail = (quatTail + 1) % QUAT_BUFFER_SIZE;
    }

    quatBuffer[quatHead] = sample;
    quatHead = (quatHead + 1) % QUAT_BUFFER_SIZE;
    quatBufferFull = (quatHead == quatTail);
}

void addToGyroBuffer(GyroSample sample)
{
    if (gyroBufferFull)
    {
        // Buffer is full, we'll overwrite oldest data
        gyroTail = (gyroTail + 1) % GYRO_BUFFER_SIZE;
    }

    gyroBuffer[gyroHead] = sample;
    gyroHead = (gyroHead + 1) % GYRO_BUFFER_SIZE;
    gyroBufferFull = (gyroHead == gyroTail);
}

void addToEulBuffer(EulerSample sample)
{
    if (eulBufferFull)
    {
        // Buffer is full, we'll overwrite oldest data
        eulTail = (eulTail + 1) % EULER_BUFFER_SIZE;
    }

    eulBuffer[eulHead] = sample;
    eulHead = (eulHead + 1) % EULER_BUFFER_SIZE;
    eulBufferFull = (eulHead == eulTail);
}

void setup()
{
    Serial.begin(115200);
    nicla::begin();                // Board initialization
    BHY2.begin(NICLA_STANDALONE);  // Start Bosch Sensor-Fusion

    // Initialize sensors at 100Hz
    quatSensor.begin(100);
    gyroSensor.begin(100);
    eulSensor.begin(100);

    // BLE setup
    if (!BLE.begin())
    {
        Serial.println("BLE initialization failed");
        while (1);
    }

    // Create a unique device name using MAC address
    String addr = BLE.address();
    String suffix = addr.substring(addr.length()-5);
    suffix.replace(":", "");
    String name = "NiclaQGE-" + suffix;

    // Configure BLE settings
    BLE.setDeviceName(name.c_str());
    BLE.setLocalName(name.c_str());
    BLE.setAdvertisedService(bleService);

    // Optimize connection parameters for reliability
    BLE.setConnectionInterval(8, 12);  // 10-15ms intervals (8-12 * 1.25ms)
    BLE.setTxPower(4);                 // Increase transmission power (0-8)
    BLE.setPreferredPhy(2, 2);         // Use 2M PHY if available

    // Add characteristics to service
    bleService.addCharacteristic(quatChar);
    bleService.addCharacteristic(gyroChar);
    bleService.addCharacteristic(eulChar);
    bleService.addCharacteristic(statusChar);

    // Add service and start advertising
    BLE.addService(bleService);
    BLE.advertise();

    Serial.print("🔊 Advertising as "); 
    Serial.println(name);
}

void loop()
{
    BHY2.update();  // Empty IMU FIFO
    
    // Track connection status
    bool isConnected = BLE.connected();
    if (!isConnected && wasConnected)
    {
        // We just disconnected
        unsigned long now = millis();
        if (now - lastReconnectAttempt > RECONNECT_INTERVAL)
        {
        Serial.println("Connection lost, restarting advertising");
        BLE.advertise();
        lastReconnectAttempt = now;
        }
    }
    wasConnected = isConnected;
    
    // Always collect data even when not connected
    uint32_t ts = millis();
    
    // ─── Quaternion data collection ──────────────────────────────────────
    if (quatSensor.dataAvailable())
    {
        QuatSample sample;
        sample.sequence = quatSeq++;
        sample.timestamp = ts;
        sample.w = quatSensor.w();
        sample.x = quatSensor.x();
        sample.y = quatSensor.y();
        sample.z = quatSensor.z();
        
        // Calculate checksum on the raw data (excluding the checksum field)
        uint8_t buffer[sizeof(QuatSample) - 1];
        memcpy(buffer, &sample, sizeof(buffer));
        sample.checksum = calculateChecksum(buffer, sizeof(buffer));
        
        // Add to buffer
        addToQuatBuffer(sample);
    }
    
    // ─── Gyroscope data collection ──────────────────────────────────────
    if (gyroSensor.dataAvailable())
    {
        GyroSample sample;
        sample.sequence = gyroSeq++;
        sample.timestamp = ts;
        sample.x = gyroSensor.x();
        sample.y = gyroSensor.y();
        sample.z = gyroSensor.z();
        
        // Calculate checksum
        uint8_t buffer[sizeof(GyroSample) - 1];
        memcpy(buffer, &sample, sizeof(buffer));
        sample.checksum = calculateChecksum(buffer, sizeof(buffer));
        
        // Add to buffer
        addToGyroBuffer(sample);
    }
    
    // ─── Euler angles data collection ─────────────────────────────────────
    if (eulSensor.dataAvailable())
    {
        EulerSample sample;
        sample.sequence = eulSeq++;
        sample.timestamp = ts;
        sample.heading = eulSensor.heading();
        sample.pitch = eulSensor.pitch();
        sample.roll = eulSensor.roll();
        
        // Calculate checksum
        uint8_t buffer[sizeof(EulerSample) - 1];
        memcpy(buffer, &sample, sizeof(buffer));
        sample.checksum = calculateChecksum(buffer, sizeof(buffer));
        
        // Add to buffer
        addToEulBuffer(sample);
    }
    
    // Transmit data if connected
    if (isConnected)
    {
        // ─── Quaternion transmission ───────────────────────────────────────
        if (quatChar.subscribed() && (quatHead != quatTail || quatBufferFull))
        {
            // Send up to 3 samples at a time to avoid congestion
            for (int i = 0; i < 3 && (quatHead != quatTail || quatBufferFull); i++)
            {
                quatChar.writeValue((uint8_t*)&quatBuffer[quatTail], sizeof(QuatSample));
                quatTail = (quatTail + 1) % QUAT_BUFFER_SIZE;
                quatBufferFull = false;
            }
        }
        
        // ─── Gyroscope transmission ───────────────────────────────────────
        if (gyroChar.subscribed() && (gyroHead != gyroTail || gyroBufferFull))
        {
            // Send up to 2 samples at a time
            for (int i = 0; i < 2 && (gyroHead != gyroTail || gyroBufferFull); i++)
            {
                gyroChar.writeValue((uint8_t*)&gyroBuffer[gyroTail], sizeof(GyroSample));
                gyroTail = (gyroTail + 1) % GYRO_BUFFER_SIZE;
                gyroBufferFull = false;
            }
        }
        
        // ─── Euler angles transmission ────────────────────────────────────
        if (eulChar.subscribed() && (eulHead != eulTail || eulBufferFull))
        {
            // Send up to 2 samples at a time
            for (int i = 0; i < 2 && (eulHead != eulTail || eulBufferFull); i++)
            {
                eulChar.writeValue((uint8_t*)&eulBuffer[eulTail], sizeof(EulerSample));
                eulTail = (eulTail + 1) % EULER_BUFFER_SIZE;
                eulBufferFull = false;
            }
        }
        
        // ─── Status updates ─────────────────────────────────────────────────
        // Update status every second
        if (millis() - lastStatusUpdate > 1000)
        {
            uint8_t statusBuffer[12];
            
            // Device uptime
            uint32_t uptime = millis();
            memcpy(statusBuffer, &uptime, 4);
            
            // Buffer fill levels (0-100%)
            statusBuffer[4] = quatBufferFull ? 100 : ((quatHead - quatTail + QUAT_BUFFER_SIZE) % QUAT_BUFFER_SIZE) * 100 / QUAT_BUFFER_SIZE;
            statusBuffer[5] = gyroBufferFull ? 100 : ((gyroHead - gyroTail + GYRO_BUFFER_SIZE) % GYRO_BUFFER_SIZE) * 100 / GYRO_BUFFER_SIZE;
            statusBuffer[6] = eulBufferFull ? 100 : ((eulHead - eulTail + EULER_BUFFER_SIZE) % EULER_BUFFER_SIZE) * 100 / EULER_BUFFER_SIZE;
            
            // Sequence counters (most significant byte only)
            statusBuffer[7] = (quatSeq >> 24) & 0xFF;
            statusBuffer[8] = (gyroSeq >> 24) & 0xFF;
            statusBuffer[9] = (eulSeq >> 24) & 0xFF;
            
            // Reserve bytes 10-11 for future use (could be battery level, error flags, etc.)
            statusBuffer[10] = 0;
            statusBuffer[11] = 0;
            
            statusChar.writeValue(statusBuffer, sizeof(statusBuffer));
            lastStatusUpdate = millis();
        }
    }
    BLE.poll();  // Handle BLE events
}
