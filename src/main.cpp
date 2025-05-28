#include <Arduino.h>
#include "SoftI2C.h"
#include "BMP280.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
//Prova di commit
JsonDocument settings;
File flightSettingsF;
String flightSettings;

SoftI2C softI2C(15, 14);
BMP280 bmp(softI2C, 0x76);
Adafruit_NeoPixel pixels(1, 16, NEO_GRB + NEO_KHZ800);

float seaPressure = 1014.11;
bool pyro = false;
float accel[3],accelTOT;
float gyro[3];
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
  ready=true;
}

void setup1(){
  while(!ready);
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
  Serial.print("Profilo di volo: ");
  Serial.print(settings["flight"].as<const char*>());
  Serial.print(", Temperatura: ");
  Serial.print(temp);
  Serial.print(" °C, Pressione: ");
  Serial.print(pressure);
  Serial.print(" hPa, Altitudine: ");
  Serial.print(altitude);
  Serial.print(" m, Continuity: ");
  Serial.print(analogRead(29));
  Serial.print(", Pyro state: ");
  Serial.print(pyro);
  Serial.print(", Measurement's time: ");
  Serial.print(timeD);
  Serial.print(" microseconds, Measurements/second: ");
  Serial.println(measurements);
  delay(100);
}

void loop1(){
  while(true){
    accel[0]=0;
    accel[1]=0;
    accel[2]=0;
    accelTOT=sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);

    gyro[0]=0;
    gyro[1]=0;
    gyro[2]=0;

    rot[0]+=gyro[0]*(micros()-last)/1000000;
    rot[1]+=gyro[1]*(micros()-last)/1000000;
    rot[2]+=gyro[2]*(micros()-last)/1000000;

    temp=(bmp.readTemperature() + /*In theory MPU's temp*/bmp.readTemperature())/2;
    pressure=bmp.readPressure();
    altitude=44330.0 * (1.0 - pow(pressure / seaPressure, 0.1903));
    timeD=(micros()-last);
    measurements=1000000/(timeD);
    last=micros();
  }
}