#include <Arduino.h>
#include <Wire.h>
#include "SoftI2C.h"
#include "BMP280.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <cmath>
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
float rot[3] = {0, 0, 0};
float temp, pressure, altitude;
float measurements, timeD;
unsigned long last;
volatile bool ready = false;

bool allow = true;

float seaLevelPressure(float altitude, float pressure)
{
  return pressure / pow(1.0 - (altitude / 44330.0), 5.255);
}

void parachute()
{
  if (flightSettings.length() > 0)
  {
    if ((String)settings["parachute"] == "pyro")
    {
      if (analogRead(29) > 100)
      {
        digitalWrite(28, HIGH);
        delay(2000);
        digitalWrite(28, LOW);
      }
    }
  }
}

void cgyro(int samples = 2000)
{
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
}

void command(String cmd)
{
  if (cmd == "reset")
  {
    allow = false;
    delay(3);
    rot[0] = 0;
    rot[1] = 0;
    rot[2] = 0;
    allow = true;
  }
  else if (cmd.startsWith("cpressure"))
  {
    seaLevelPressure(cmd.substring(9).toFloat(), bmp.readPressure());
  }
  else if (cmd == "cgyro")
  {
    cgyro();
  }
  else if (cmd == "settings")
  {
    Serial.println(flightSettings);
  }
}

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  softI2C.begin();
  delay(200);
  pixels.setPixelColor(0, pixels.Color(0, 70, 70));
  pixels.show();
  delay(1800);

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

  cgyro();

  Serial.println("Calibrated gyroscope:");
  Serial.print("X offset: ");
  Serial.println(gyro_offset[0]);
  Serial.print("Y offset: ");
  Serial.println(gyro_offset[1]);
  Serial.print("Z offset: ");
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
    cmd.replace("\r", "");
    cmd.toLowerCase();
    command(cmd);
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
  while (!allow)
    ;
  // Richiedi i 14 byte in un colpo solo (più veloce e compatto)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_DATA_START);
  Wire.endTransmission(false); // no stop
  Wire.requestFrom(MPU6050_ADDR, 14);

  // Accelerometro (6 byte)
  for (int i = 0; i < 3; i++)
  {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    accel[i] = raw / ACCEL_SCALE;
  }

  accelTOT = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

  Wire.read();
  Wire.read(); // salta la temperatura (2 byte)

  float dt = (micros() - last) / 1000000.0f; // secondi

  // Altimetro (solo una lettura ogni 10-50ms sarebbe sufficiente in molti casi)
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = 44330.0f * (1.0f - powf(pressure / seaPressure, 0.1903f));

  // Giroscopio (6 byte)
  for (int i = 0; i < 3; i++)
  {
    int16_t raw = (Wire.read() << 8) | Wire.read();
    gyro[i] = (raw / GYRO_SCALE) - gyro_offset[i];
    rot[i] += gyro[i] * dt;
    if (rot[i] >= 360.0f)
      rot[i] -= 360.0f;
    else if (rot[i] < 0.0f)
      rot[i] += 360.0f;
  }

  timeD = micros() - last;
  measurements = 1000000.0f / timeD;
  last = micros();
}
