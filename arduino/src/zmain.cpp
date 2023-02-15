
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#define BLINK_INTERVAL 200 // blink every 200ms

// for feather32u4 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// CONFIG BETWEEN RX OR TX MODULE
// #define TRANSMITTER
#define RECEIVER

unsigned long last_blink;
int led_state;

#ifdef TRANSMITTER
  #include "MPU9250.h"
  MPU9250 mpu;

  void updateMPU();
  void setupMPU();
#endif

#ifdef RECEIVER
  void receiveLoRa();
#endif

//this structure is needed by hatire
struct  {
  int16_t  Begin;   // 2  Debut
  uint16_t Cpt;      // 2  Compteur trame or Code
  float    gyro[3];   // 12 [Y, P, R]    gyro
  float    acc[3];    // 12 [x, y, z]    Acc
  int16_t  End;      // 2  Fin
} hat;

void setup ()
{
  // setup LED Pin
  pinMode(LED_BUILTIN, OUTPUT);
  last_blink = millis();
  led_state = HIGH;
  digitalWrite(LED_BUILTIN, led_state);

  // setup sensor
  #ifdef RECEIVER
    Serial.begin(115200);
    while (!Serial);
  #endif

  // header frame for hatire
  hat.Begin=0xAAAA;
  // Frame Number or Error code
  hat.Cpt=0;
  // footer frame for hatire
  hat.End=0x5555;

  // setup mpu sensor
  #ifdef TRANSMITTER
    setupMPU();
  #endif

  // Setup LoRa
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT); // set CS, reset, IRQ pin

  if (!LoRa.begin(434E6)) {
    // Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(7); // ranges from 6-12,default 7 see API docs
  LoRa.setSyncWord(0xF2);     // ranges from 0-0xFF, default 0x34, see API docs
}  // end of setup

void loop ()
{
  // get time
  unsigned long curr_millis = millis();

  // blink led to show we are still running
  if ((curr_millis - last_blink) > BLINK_INTERVAL) {
    // flip led state
    led_state = led_state ? LOW : HIGH;
    digitalWrite(LED_BUILTIN, led_state);
    last_blink = curr_millis;
  }

  // if TX
  #ifdef TRANSMITTER
    updateMPU();
  #endif

  // if RX
  #ifdef RECEIVER
    receiveLoRa();
  #endif   
}  // end of loop


#ifdef RECEIVER
void receiveLoRa() 
{
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    byte * structStart;
    structStart = reinterpret_cast <byte *> (&hat);
    byte buffer[30];
    // read packet
    while (LoRa.available()) {
      LoRa.readBytes(buffer, 30);
    }
    // flush serial
    while (LoRa.available() > 0) {
      byte dumpTheData = LoRa.read();
    }

    // read into hat buffer, ideally we should be able to transmit it straight, but lets see if this works, then try to optimize
    for (byte n = 0; n < 30; n++) {
        *(structStart + n) = buffer[n];
    }
    Serial.write((byte*)&hat, 30);
    // Serial.print(hat.gyro[0]);
    // Serial.print(", ");
    // Serial.print(hat.gyro[1]);
    // Serial.print(", ");
    // Serial.print(hat.gyro[2]);
    // Serial.print(", ");
    // Serial.println(hat.Cpt);
    // print RSSI of packet
    // Serial.print("' with RSSI ");
    // Serial.println(LoRa.packetRssi());
  }
}
#endif


#ifdef TRANSMITTER
void setupMPU ()
{
  Wire.begin();

  mpu.setup(0x68);
  // set here the mag. declination for your country  http://www.magnetic-declination.com
  mpu.setMagneticDeclination(4.033);

  /* 1) load the calibration script and open the terminal, leave the MPU on the table without move it;
  2) the magnetometer now, it says to draw some eight. Move it drawing a sphere, fast you move more data you'll get.
  3) replace my values, with yours. DO NOT use the accelerometer values.
  */    
  //mpu.setAccBias(0, +13.85);
  //mpu.setAccBias(1, +68.24);
  //mpu.setAccBias(2, -206.36);
  mpu.setGyroBias(+1.29, -1.43, -4.37);
  mpu.setMagBias(+160.77, +103.95, -146.86);
  mpu.setMagScale(+1.19, +0.90, +0.95);
}
void updateMPU ()
{
  if (mpu.update())
  {
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 25)
    {

      // mpu.print();

      // THIS IS COMMENTED, use it to read values of pitch, yaw, roll.
      /*Serial.print("roll(x-forward (north)): ");
      Serial.print(mpu.getRoll());
      Serial.println();
      Serial.print("pitch(y-right (east)): ");
      Serial.print(mpu.getPitch());
      Serial.println();
      Serial.print("yaw(z-down (down)): ");
      Serial.print(mpu.getYaw());
      Serial.println();*/

      /* hat.acc[0]=a[0];
      hat.acc[1]=a[1];
      hat.acc[2]=a[2];*/

      // comment these if you want to read pitch, yaw, roll
      hat.gyro[0] = mpu.getYaw();
      hat.gyro[1] = mpu.getPitch();
      hat.gyro[2] = mpu.getRoll();

      prev_ms = millis();

      // Send HAT  Frame to  PC
      LoRa.beginPacket();
      LoRa.write((byte *)&hat, 30);
      LoRa.endPacket();

      // transmitLora();
      hat.Cpt++;
      if (hat.Cpt > 999)
      {
        hat.Cpt = 0;
      }
    }
  }
}
#endif