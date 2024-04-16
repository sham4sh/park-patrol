#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <TFLI2C.h>   
#include <TinyGPSPlus.h> 
#include <SoftwareSerial.h> 

#include <ArduinoSTL.h>
#include <map>
#include <utility>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

static const int RXPin = 0, TXPin = 1; 
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps; 
SoftwareSerial ss(RXPin, TXPin);

Adafruit_BMP3XX bmp;

TFLI2C tflI2C;
 
int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // Use this default I2C address

// Parking spaces occupied states - 0 free, 1 occupied
int spaces[2][13] = 
{
         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 
         { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};

std::map<std::pair<int, int>, std::pair<double, double>> locations; 

/** Finds the closest mapped parking space by Pythagoras' Theorem and returns the corresponding index of the spot matrix.
 * @param x latitude
 * @param y longitude
 * */  
std::pair<int, int> findSpot(double x, double y)
{
    double minDist = 10000;
    std::pair<int, int> returnPair = {0, 0};
    for (auto& item : locations) 
    {
    
        double xDist = x - item.second.first;
        double yDist = y - item.second.second;

        double linearDist = sqrt((xDist * xDist) + (yDist * yDist));

        if (linearDist < minDist)
        {
            minDist = linearDist;
            returnPair = item.first;
        }
    }

    return returnPair;

}

void displayInfo() { 
// Displaying Google Maps link with latitude and longitude information if GPS location is valid 

if (gps.location.isValid()) { 

Serial.print("http://www.google.com/maps/place/"); 
Serial.print(gps.location.lat(), 6); 
Serial.print(F(",")); 
Serial.println(gps.location.lng(), 6); } else { Serial.print(F("INVALID"));
} // Displaying date and time information if valid 



if (gps.date.isValid()) { 

// Displaying date in MM/DD/YYYY format // If the date is invalid, it prints "INVALID" 

} else 

{ Serial.print(F("INVALID")); } // Displaying time in HH:MM:SS format if valid // If the time is invalid, it prints "INVALID" 

}


void setup() {
  Wire.begin();

  Serial.begin(9600);

  ss.begin(GPSBaud); 
  Serial.println(F("DeviceExample.ino")); // Printing information for demonstration and version of TinyGPSPlus library // by Mikal Hart 

  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    //while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Mapped coordinates for every parking spot. The key corresponds to the spot's index in the occupied state matrix.
  locations[{ 0, 0 }] = {1232.12, 1923.4}; 
  locations[{ 0, 1 }] = {1232.12, 1923.4}; 
  locations[{ 0, 2 }] = {1232.12, 1232.12}; 
  locations[{ 0, 3 }] = {1232.12, 1923.4};
  locations[{ 0, 4 }] = {1232.12, 1923.4}; 
  locations[{ 0, 5 }] = {1232.12, 1923.4}; 
  locations[{ 0, 6 }] = {1232.12, 1923.4}; 
  locations[{ 0, 7 }] = {1232.12, 1923.4}; 
  locations[{ 0, 8 }] = {1232.12, 1923.4}; 
  locations[{ 0, 9 }] = {1232.12, 1232.12}; 
  locations[{ 0, 10 }] = {1232.12, 1923.4};
  locations[{ 0, 11 }] = {1232.12, 1923.4}; 
  locations[{ 0, 12 }] = {1232.12, 1923.4}; 

  locations[{ 1, 0 }] = {1232.12, 1923.4}; 
  locations[{ 1, 1 }] = {1232.12, 1923.4}; 
  locations[{ 1, 2 }] = {1232.12, 1232.12}; 
  locations[{ 1, 3 }] = {1232.12, 1923.4};
  locations[{ 1, 4 }] = {1232.12, 1923.4}; 
  locations[{ 1, 5 }] = {1232.12, 1923.4}; 
  locations[{ 1, 6 }] = {1232.12, 1923.4}; 
  locations[{ 1, 7 }] = {1232.12, 1923.4}; 
  locations[{ 1, 8 }] = {1232.12, 1923.4}; 
  locations[{ 1, 9 }] = {1232.12, 1232.12}; 
  locations[{ 1, 10 }] = {1232.12, 1923.4};
  locations[{ 1, 11 }] = {1232.12, 1923.4}; 
  locations[{ 1, 12 }] = {1232.12, 1923.4}; 
}

void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }else{
    Serial.println("Pressure read success");
  }
  /**
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  
  */

 if(tflI2C.getData(tfDist, tfAddr)){
        Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
  }else{
    Serial.println("LiDAR read failed");
  }

  

  if (gps.encode(ss.read())) { 
    Serial.println("GPS found");
    displayInfo(); 
  }else{
    Serial.println("GPS not found");
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) { 
    Serial.println(F("No GPS detected: check wiring.")); 
  while(true); }

  delay(200);
}

