// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <PID_v1.h>

#define DHT1PIN 2     // Digital pin connected to the DHT sensor 
#define DHT2PIN 3     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT22     // DHT 22 (AM2302)

//Relay control pins
#define RELAY1 5
#define RELAY2 6
//Set the frequency of updating the relay here, in minutes. The larger the value, the less likely you will have rapid on/off cycling
#define RELAYINTERVAL 1

//Set the ventilation threshold of the DEWPOINT in degrees celcius. I recommend it being greater than 1, as that seems to be the margin of error for these sensors
#define VENTTHRESHOLD 1.5

//Set cutoff temp for ventilation, celcius
#define TEMPCUTOFF 15

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview
DHT_Unified dht1(DHT1PIN, DHTTYPE);
DHT_Unified dht2(DHT2PIN, DHTTYPE);

uint32_t delayMS = 5000;
unsigned long WindowSize = 61000;

//Globals. We use longs because we are counting seconds, 16bit would overrun
long relayCount=0;
long minutes=0;
long subcount=29;
int vent=0;

float ambient_temp = 0;
float chamber_temp = 0;
float ambient_humid = 0;
float chamber_humid = 0;
float target_temp = 50;
const unsigned long minOnTime = 10000;  // min 10s ON
const unsigned long minOffTime = 10000; // min 10s OFF
unsigned long lastSwitchTime = 0;
bool relayOn = false;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the initial tuning parameters (Period of 10.3 minutes)
double Kp=, Ki=, Kd=;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);


unsigned long windowStartTime;

void setup() {
  Serial.begin(9600);
  // Initialize device.  
  // Serial.println(F("DHT22 Unified Sensor Example"));
  delay(1000);  //Make sure sensor had enough time to turn on.

  dht1.begin();
  // Print temperature sensor details.
  sensor_t sensor1;
  dht1.temperature().getSensor(&sensor1);
  // Serial.println(F("------------------------------------"));
  // Serial.println(F("Temperature Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor1.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor1.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor1.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor1.max_value); Serial.println(F("°C"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor1.min_value); Serial.println(F("°C"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor1.resolution); Serial.println(F("°C"));
  // Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht1.humidity().getSensor(&sensor1);
  // Serial.println(F("Humidity Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor1.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor1.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor1.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor1.max_value); Serial.println(F("%"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor1.min_value); Serial.println(F("%"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor1.resolution); Serial.println(F("%"));
  // Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  // delayMS = sensor1.min_delay / 1000;
  // delayMS *= 5;

  // Initialize second DHT sensor
  dht2.begin();
  // Print temperature sensor details.
  sensor_t sensor2;
  dht2.temperature().getSensor(&sensor2);
  // Serial.println(F("------------------------------------"));
  // Serial.println(F("Temperature Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor2.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor2.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor2.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor2.max_value); Serial.println(F("°C"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor2.min_value); Serial.println(F("°C"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor2.resolution); Serial.println(F("°C"));
  // Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht2.humidity().getSensor(&sensor2);
  // Serial.println(F("Humidity Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor2.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor2.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor2.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor2.max_value); Serial.println(F("%"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor2.min_value); Serial.println(F("%"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor2.resolution); Serial.println(F("%"));
  // Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  //delayMS = sensor2.min_delay / 1000; it's the same sensor.

  //Initialize Relays, set to OFF (which is HIGH, arduino set to LOW at boot)
  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
  digitalWrite(RELAY1,HIGH);
  digitalWrite(RELAY2,HIGH);


  windowStartTime = millis();
  //initialize the variables we're linked to
  Setpoint = target_temp;
  //tell the PID to range between 0 and 255 (can't use the full window size. 
  myPID.SetOutputLimits(0, 255);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  subcount++;

  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht1.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    // Serial.println(F("Error reading temperature!"));
  }
  else {
    // Serial.print(F("Ambient Temperature: "));
    // Serial.print(event.temperature);
    // Serial.println(F("°C"));
    float fahrenheitTemperature = (event.temperature * 1.8) + 32.0;
    // Serial.print(fahrenheitTemperature);
    // Serial.println(F("°F"));
    ambient_temp = fahrenheitTemperature;
  }
  // Get humidity event and print its value.
  dht1.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    // Serial.println(F("Error reading humidity!"));
  }
  else {
    // Serial.print(F("Ambient Humidity: "));
    // Serial.print(event.relative_humidity);
    // Serial.println(F("%"));
    ambient_humid = event.relative_humidity;
  }
  float dpIn = dewpoint(event.temperature,event.relative_humidity);
  if (isnan(dpIn)) {
    // Serial.println(F("Error calculating dew point!"));
  }
  else {
    // Serial.print(F("Ambient Dew point: "));
    // Serial.print(dpIn);
    // Serial.println(F("°C"));
    float fahrenheitDewpoint = (dpIn * 1.8) + 32.0;
    // Serial.print(fahrenheitDewpoint);
    // Serial.println(F("°F"));
  }

  dht2.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    // Serial.println(F("Error reading temperature!"));
  }
  else {
    // Serial.print(F("Chamber Temperature: "));
    // Serial.print(event.temperature);
    // Serial.println(F("°C"));
    float fahrenheitTemperature = (event.temperature * 1.8) + 32.0;
    // Serial.print(fahrenheitTemperature);
    // Serial.println(F("°F"));
    chamber_temp = fahrenheitTemperature;
  }
  // Get humidity event and print its value.
  dht2.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    // Serial.println(F("Error reading humidity!"));
  }
  else {
    // Serial.print(F("Chamber Humidity: "));
    // Serial.print(event.relative_humidity);
    // Serial.println(F("%"));
    chamber_humid = event.relative_humidity;
  }
  
  // Serial.println("--------------");
  // Serial.println();

  // PID Algorithm to decide if Peltier should be energized.
  Input = chamber_temp;
  myPID.Compute();

  /************************************************
  * turn the output pin on/off based on pid output
  ************************************************/ 
  // The value calculated by the PID that represents "how hard" the system needs to work 
  unsigned long scaledOutput = (unsigned long)(Output * (WindowSize / 255.0) + 0.5);
  if (scaledOutput > WindowSize) 
    scaledOutput = WindowSize;

  unsigned long elapsed = millis() - windowStartTime;
  while (elapsed > WindowSize) {
    windowStartTime += WindowSize;
    elapsed = millis() - windowStartTime;
  }

  updateRelay(scaledOutput, elapsed);

  // Serial.print("AmbientTemp:");
  Serial.print(ambient_temp);
  // Serial.print(", ChamberTemp:"); 
  Serial.print(","); 
  Serial.print(chamber_temp);
  // Serial.print(", AmbientHumidity:"); 
  // Serial.print(ambient_humid);
  // Serial.print(", ChamberHumidity:"); 
  // Serial.print(chamber_humid);
  Serial.print(",");
  // Serial.print(", TargetTemp:"); 
  Serial.print(target_temp);
  Serial.print(","); 
  // Serial.print(", PeltierState:");
  Serial.print(relayOn);
  Serial.print(",");
  Serial.print(elapsed);
    Serial.print(",");
  Serial.println(Output);
}

//quick dp approximation, http://en.wikipedia.org/wiki/Dew_point
float dewpoint(float temperature, float humidity)
{
  float a = 17.271;
  float b = 237.7;
  float temp = (a * temperature) / (b + temperature) + log((double) humidity/100);
  double Td = (b * temp) / (a - temp);
  return Td;
}

void updateRelay(unsigned long scaledOutput, unsigned long elapsed) {
  unsigned long now = millis();
  if (scaledOutput > elapsed) { // ideally should be ON
    if (!relayOn && (now - lastSwitchTime >= minOffTime)) {
      digitalWrite(RELAY2, LOW);
      relayOn = true;
      lastSwitchTime = now;
    }
    // else maintain ON state if already on and minOnTime not expired
  } else { // ideally OFF
    if (relayOn && (now - lastSwitchTime >= minOnTime)) {
      digitalWrite(RELAY2, HIGH);
      relayOn = false;
      lastSwitchTime = now;
    }
    // else maintain OFF if already off and minOffTime not expired
  }
}

