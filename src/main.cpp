#include <Arduino.h>
#include <Wire.h>
#include "SoftI2C.h"
#include "BMP280.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>

JsonDocument settings;
File flightSettingsF;
String flightSettings;

SoftI2C softI2C(15, 14);
BMP280 bmp(softI2C, 0x76);
Adafruit_NeoPixel pixels(1, 16, NEO_GRB + NEO_KHZ800);

// MPU6050 I2C
const uint8_t MPU6050_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_DATA_START = 0x3B; // ACCEL_XOUT_H

// Scale factors
const float ACCEL_SCALE = 2048.0; // ±16g
const float GYRO_SCALE = 16.4;    // ±2000°/s

float seaPressure = 1014.11;
bool pyro = false;
float accel[3], accelTOT;
float gyro[3];
float gyro_offset[3] = {0, 0, 0};
float rot[3];
float temp, pressure, altitude;
float measurements, timeD;
unsigned long last;
volatile bool ready = false;

float seaLevelPressure(float altitude, float pressure)
{
  return pressure / pow(1.0 - (altitude / 44330.0), 5.255);
}

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  softI2C.begin();
  delay(2000);

  SPI1.setRX(8);
  SPI1.setTX(11);
  SPI1.setSCK(10);
  if (!SD.begin(9, SPI1))
  {
    Serial.println("MicroSD not working");
    pixels.setPixelColor(0, pixels.Color(70, 0, 0));
    pixels.show();
    while (1)
      delay(10);
  }

  flightSettingsF = SD.open("flight.json");
  while (flightSettingsF.available())
  {
    flightSettings += (char)flightSettingsF.read();
  }
  flightSettingsF.close();
  Serial.println("Flight settings:");
  Serial.println(flightSettings);
  Serial.println("\n\n\n");
  deserializeJson(settings, flightSettings);
  if (!bmp.begin())
  {
    Serial.println("BMP280 non trovato!");
    pixels.setPixelColor(0, pixels.Color(70, 0, 70));
    pixels.show();
    while (1)
      delay(10);
  }
  Serial.println("BMP280 inizializzato!");
  pixels.setPixelColor(0, pixels.Color(0, 70, 0));
  pixels.show();
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);

  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();
  Wire.setClock(400000); // I2C veloce

  // Disattiva sleep mode
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  // Imposta accelerometro a ±16g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_CONFIG);
  Wire.write(0x18); // AFS_SEL = 3
  Wire.endTransmission();
  delay(10);

  // Imposta giroscopio a ±2000°/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(0x18); // FS_SEL = 3
  Wire.endTransmission();
  delay(10);

  const int samples = 500;
  long sum[3] = {0, 0, 0};

  for (int i = 0; i < samples; i++)
  {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_DATA_START);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    Wire.read();
    Wire.read(); // accelX
    Wire.read();
    Wire.read(); // accelY
    Wire.read();
    Wire.read(); // accelZ
    Wire.read();
    Wire.read(); // skip temp

    for (int j = 0; j < 3; j++)
    {
      int16_t raw = (Wire.read() << 8) | Wire.read();
      sum[j] += raw;
    }
    delay(2);
  }

  for (int i = 0; i < 3; i++)
    gyro_offset[i] = (sum[i] / (float)samples) / GYRO_SCALE;

  Serial.println("Giroscopio calibrato:");
  Serial.print("Offset X: ");
  Serial.println(gyro_offset[0]);
  Serial.print("Offset Y: ");
  Serial.println(gyro_offset[1]);
  Serial.print("Offset Z: ");
  Serial.println(gyro_offset[2]);

  ready = true;
}

void setup1()
{
  while (!ready)
    ;
}

void loop()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "settings\r")
    {
      Serial.println(flightSettings);
    }
    else
    {
      seaPressure = seaLevelPressure(cmd.toFloat(), pressure);
      Serial.print("Sea pressure updated to ");
      Serial.print(seaPressure);
      Serial.print(" hPa (Altitude received: ");
      Serial.print(cmd);
      Serial.println(")");
    }
  }
  pixels.setPixelColor(0, pixels.Color(0, 70, 0));
  if (analogRead(29) > 60)
    pixels.setPixelColor(0, pixels.Color(0, 0, 70));
  pixels.show();
  Serial.print("Temperatura: ");
  Serial.print(temp);
  Serial.print(" °C, Pressione: ");
  Serial.print(pressure);
  Serial.print(" hPa, Altitudine: ");
  Serial.print(altitude);
  Serial.print(", Measurement's time: ");
  Serial.print(timeD);
  Serial.print(" microseconds, Measurements/second: ");
  Serial.print(measurements);
  Serial.print(", Accelerations [g]: (");
  Serial.print(accel[0]);
  Serial.print(",");
  Serial.print(accel[1]);
  Serial.print(",");
  Serial.print(accel[2]);
  Serial.print("), [deg/s]: (");
  Serial.print(gyro[0]);
  Serial.print(",");
  Serial.print(gyro[1]);
  Serial.print(",");
  Serial.print(gyro[2]);
  Serial.print("), Rotation [deg]: (");
  Serial.print(rot[0]);
  Serial.print(",");
  Serial.print(rot[1]);
  Serial.print(",");
  Serial.print(rot[2]);
  Serial.println(")");
  delay(400);
}

void loop1()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_DATA_START);
  Wire.endTransmission(false); // no stop

  Wire.requestFrom(MPU6050_ADDR, 14);

  for (int i = 0; i < 3; i++)
  {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    accel[i] = raw / ACCEL_SCALE;
  }

  accelTOT = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

  Wire.read();
  Wire.read(); // Skips temperature

  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = 44330.0 * (1.0 - pow(pressure / seaPressure, 0.1903));

  float dt = (micros() - last) / 1000000.0; // secondi
  for (int i = 0; i < 3; i++)
  {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    gyro[i] = (raw / GYRO_SCALE) - gyro_offset[i];
    rot[i] += gyro[i] * dt;
  }

  timeD = (micros() - last);
  measurements = 1000000 / (timeD);
  last = micros();
}