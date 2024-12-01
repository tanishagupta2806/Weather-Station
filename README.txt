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
