#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"

//Wifi
#include <WiFi.h>
#include <HTTPClient.h>
#include "ThingSpeak.h"                 

const char *ssid =  "TANISHA 2005";                                    // replace with your wifi ssid and wpa2 key
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
