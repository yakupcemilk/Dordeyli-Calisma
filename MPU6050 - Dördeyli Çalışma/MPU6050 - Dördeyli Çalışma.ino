#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
/* ======================================================
  hata ve gimbal lock için
  http://arduino.cc/forum/index.php/topic,109987.0.html
  http://code.google.com/p/arduino/issues/detail?id=958
 * ======================================================*/

#define OUTPUT_READABLE_QUATERNION
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL


#define LED_PIN 13 
bool blinkState = false;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           // [w, x, y, z]         dördeyler
VectorInt16 aa;         // [x, y, z]            ivmeölçer ölçümleri
VectorInt16 aaWorld;    // [x, y, z]            yerçekimi ivmesiz (9,81m/s çıkarılarak optimize edilmiş) ivmeölçer ölçümleri
VectorFloat gravity;    // [x, y, z]            yerçekimi klasik vektörleri
float euler[3];         // [psi, theta, phi]    Euler açıları
float ypr[3];           // [yaw, pitch, roll]   x, y ve z direksiyonlarındaki ana hareketler

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               VERİ KESİNTİSİNİ TESPİT ETME               ===
// ================================================================

volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      İLK KURULUM                         ===
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("I2C cihazları başlatılıyor..."));
    mpu.initialize();

    Serial.println(F("Cihaz bağlantıları test edilmektedir..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 bağlanmıştır") : F("MPU6050 bağlanamamıştır."));

    Serial.println(F("\nDevam etmek için herhangi bir tuşa basınız: "));
    while (Serial.available() && Serial.read()); 
    while (!Serial.available());                 
    while (Serial.available() && Serial.read()); 

    Serial.println(F("DMP başlatılıyor..."));
    devStatus = mpu.dmpInitialize();

//En düşük hassaslıkta başlangıç ayarları
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        Serial.println(F("DMP'yi etkinleştirme..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Veri kesintizi analiz edilmektedir (Arduino çıktısı 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP başlatıldı, girdi bekleniyor..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP başlatılamadı (hata kodu "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    ANA PROGRAM KISMI                     ===
// ================================================================

void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO taşma yapmaktadır!")); //bu hatayı alıyorsan ilkokuldan tekrar başla

    } else if (mpuIntStatus & 0x02) {

        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("dordey\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
