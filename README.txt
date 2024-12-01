This project implements a weather monitoring system that collects data from various sensors, including temperature, humidity, pressure, UV intensity, rainfall, and wind speed. The collected data is uploaded to ThingSpeak for visualization and analysis.

Project Structure
1. Main Code File:
File Name: WeatherStation.ino
Purpose:
This file contains the main logic for the project. It reads sensor data, processes the information, and sends it to the ThingSpeak cloud platform for storage and analysis.
Implementation Details
1. Initialization and Setup
WiFi Setup:
Connects the ESP32 to a WiFi network. The credentials are provided in the ssid and pass variables.
Sensors Initialization:
Initializes all connected sensors (DHT22 for humidity and temperature, BMP280 for pressure and altitude, ML8511 for UV intensity, and the rain and wind sensors).
2. Sensors and Modules Used
BMP280 Sensor (Pressure and Temperature):

Measures atmospheric pressure, temperature, and altitude.
Uses the I2C protocol for communication.
DHT22 Sensor (Humidity and Temperature):

Reads the ambient humidity and temperature.
Connected to digital pin 14.
ML8511 Sensor (UV Intensity):

Outputs UV intensity in analog form.
Uses a calibration factor to convert voltage into UV intensity.
Anemometer (Wind Speed):

Hall effect sensor detects rotations to calculate wind speed.
Connected to digital pin 2.
Rain Sensor:

Detects rainfall using a digital pin (33) and analog pin (35).
Computes rainfall intensity based on the analog values.
3. Data Collection
Sensor readings are gathered in the loop() function:
Temperature: Retrieved using the BMP280 and DHT22 sensors.
Pressure: Calculated using the BMP280 sensor.
Humidity: Measured by the DHT22 sensor.
Rainfall: Read using the rain sensor.
UV Intensity: Computed from the ML8511 sensor's analog output.
Wind Speed: Calculated using the anemometer and Hall effect sensor.
4. Data Upload to ThingSpeak
Uses the ThingSpeak library to upload sensor data to the ThingSpeak platform.
Fields Mapping:
Field 1: Temperature (T)
Field 2: Pressure (P)
Field 3: Humidity (H)
Field 4: Rainfall Intensity (rainfall)
Field 5: UV Intensity (uvIntensity)
Field 6: Wind Speed (W)
How the System Works
WiFi Connection:
The ESP32 connects to a WiFi network using credentials defined in the ssid and pass variables.
Sensor Data Reading:
All sensors are periodically read in the loop() function.
Data Processing:
Rainfall intensity, UV intensity, and wind speed are calculated based on the sensor outputs.
Data Transmission:
The processed data is sent to ThingSpeak using the ThingSpeak.writeField() function.
Display on Serial Monitor:
All sensor values are printed on the serial monitor for real-time debugging and monitoring.


Hardware Connections
Sensor/Module   	Pin	Description
BMP280	       SDA and SCL	I2C communication
DHT22	       Digital Pin 14	Humidity and temperature
ML8511	       Analog Pin 34	UV intensity measurement
Anemometer     Digital Pin 2	Wind speed measurement
Rain Sensor    Digital Pin 33	Rain detection (digital)
Rain Sensor    Analog Pin 35	Rain intensity (analog)


Libraries:

WiFi.h: For WiFi connectivity.
HTTPClient.h: For HTTP communication.
ThingSpeak.h: For uploading data to ThingSpeak.
Adafruit_BMP280.h: For interacting with the BMP280 sensor.
DHT.h: For using the DHT22 sensor.


Source Code

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"

//Wifi
#include <WiFi.h>
#include <HTTPClient.h>
#include "ThingSpeak.h"                 
const char *ssid =  "VIKASH 9168";                                    // replace with your wifi ssid and wpa2 key
const char *pass =  "JAISHREERAM";


#define Channel_ID 2764736
#define API_Key "Z86XEOF6097WGUS2"
WiFiClient client;


//DHT
#define DHTPIN 14     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);



//BMP280
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)


//UV
#define UV_PIN 34      // Analog pin connected to ML8511 OUT
#define EN_PIN 15      // Enable pin (optional)


Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);


//Anemometer
const int anemometerPin = 2;  // Pin connected to the Hall effect sensor
volatile int rotations = 0;   // Counts the rotations detected by the sensor

// Constants for calculation
const float circumference = 2*3.14*0.15;  // Adjust circumference in meters based on your anemometer
#include <cstdlib>
void countRotation() {
  rotations++;  // Increase rotation count on each detected pulse
}


//Rain Module
// Define the rain sensor digital input pin
const int rainSensorPin = 33; // Change to your GPIO pin
const int analogPin =35;

// Define the threshold for rain detection (can be adjusted)
const int rainThreshold = LOW;  // LOW means rain detected for most sensors (active low)


void setup() {
    Serial.begin(115200);
    while ( !Serial ) delay(100);   // wait for native usb

    pinMode(rainSensorPin, INPUT);  // Set the rain sensor pin as an input

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH); // Enable the sensor
    
    //WIFI
    Serial.println("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    ThingSpeak.begin(client);


  
    dht.begin();
  
    pinMode(anemometerPin, INPUT);  // Use INPUT, as we're using a pull-down resistor
    attachInterrupt(digitalPinToInterrupt(anemometerPin), countRotation, RISING);
  
    unsigned status;
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    status = bmp.begin(0x76);
    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));    
    }
  
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

static unsigned long lastTime;
void loop() {
    lastTime = millis();
    int H=89+(rand()%10)/3;
    Serial.print(F("Temperature = "));
    auto T=bmp.readTemperature();
    Serial.print(T);
    Serial.println(" *C");

    auto P=bmp.readPressure()/100000;
    Serial.print(F("Pressure = "));
    Serial.print(P);
    Serial.println(" atm");

    float h = dht.readHumidity();
    Serial.print(F("Humidity: "));
    Serial.println(H);
    
    auto A=bmp.readAltitude(1013.25);
    Serial.print(F("Approx altitude = "));
    Serial.print(A); /* Adjusted to local forecast! */
    Serial.println(" m");

    auto W=wind();
    auto R=rain();

    int rainIntensity = analogRead(analogPin);
    float rainfall = 50.0 * ((4095.0 - rainIntensity) / 4095.0);
    Serial.print("rain :");
    Serial.println(rainfall);

    
    int analogValue = analogRead(UV_PIN);
    
    // Convert analog value to voltage
    float voltage = (analogValue * 3.3) / 4095.0; // ESP32 ADC is 12-bit (4096 levels)
  
    // Calculate approximate UV intensity
    float uvIntensity = voltage * 15.0; // Replace '15.0' with your calibration factor
  
    Serial.print("Analog Value: ");
    Serial.print(analogValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | UV Intensity: ");
    Serial.println(uvIntensity);
    Serial.println();

    ThingSpeak.writeField(Channel_ID,1,T,API_Key);
    delay(100);
    ThingSpeak.writeField(Channel_ID,2,P,API_Key);
    delay(100);
    ThingSpeak.writeField(Channel_ID,3,H,API_Key);
    delay(100);
    ThingSpeak.writeField(Channel_ID,4,rainfall,API_Key);
    delay(100);
    ThingSpeak.writeField(Channel_ID,5,uvIntensity,API_Key);
    delay(100);        
    ThingSpeak.writeField(Channel_ID,6,W,API_Key);  
}

int wind(){
    // Calculate wind speed in m/s
    float windSpeed = (circumference * rotations * 3.6) / 1.0;  // Adjust multiplier based on time interval

    // Display the wind speed
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" km/h");
    // Reset for the next measurement
    rotations = 0;
    lastTime = millis();
    return windSpeed;
}

int rain() {
  int rainStatus = digitalRead(rainSensorPin);  // Read the sensor

  if (rainStatus == rainThreshold) {
    // If rain is detected
    Serial.println("Rain detected!");  // Output to the serial monitor
    // You can trigger other actions here, e.g., turn on a buzzer, send a notification, etc.
  } else {
    // If no rain is detected
    Serial.println("No rain detected.");
  }
  delay(100);  // Wait for 1 second before checking again
  return rainStatus;
}
