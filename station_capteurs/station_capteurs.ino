#include <Adafruit_DPS310.h>
#include <Wire.h>
#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h"

Adafruit_DPS310 dps;

int windSpeedPin = 27;
int windDirectionPin = 35;
int rainfallPin = 19;
int pin_TX = 2;
int pin_RX = 15;

SFEWeatherMeterKit weatherMeterKit(windDirectionPin, windSpeedPin, rainfallPin);

struct HumiditySensorData {
  float humidity;
  float temperature;
};

struct PressureSensorData {
  float pressure;
  float temperature;
};

struct SensorData {
  float light = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
  const char* windDirection = "N";
  float windSpeed = 0.0;
  float rainfall = 0.0;
  float rainfallRate = 0.0;
};

static float getLight();
static HumiditySensorData getHumiditySensorData();
static PressureSensorData getPressureSensorData();
static const char* getWindDirection();
static float getWindSpeed();
static void updateSensorData();


SensorData data;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);  // Attend que le port série soit prêt (utile sur certains PC)

  if (!dps.begin_I2C()) {
    Serial.println("Erreur : capteur DPS310 non détecté !");
    while (1); // Boucle infinie si le capteur n'est pas trouvé
  }

  Serial.println("Capteur DPS310 détecté.");

  #ifdef SFE_WMK_PLAFTORM_UNKNOWN
    // The platform you're using hasn't been added to the library, so the
    // expected ADC values have been calculated assuming a 10k pullup resistor
    // and a perfectly linear 16-bit ADC. Your ADC likely has a different
    // resolution, so you'll need to specify it here:
    weatherMeterKit.setADCResolutionBits(10);
    
    Serial.println(F("Unknown platform! Please edit the code with your ADC resolution!"));
    Serial.println();
  #endif

  // Begin weather meter kit
  weatherMeterKit.begin();
}

static PressureSensorData getPressureSensorData() {
  sensors_event_t tempEvent, pressureEvent;

  // Lire température et pression
  dps.getEvents(&tempEvent, &pressureEvent);

  PressureSensorData result;
  result.pressure = pressureEvent.pressure * 100; // hPa → Pa
  result.temperature = tempEvent.temperature;
  return result;
}

static float getLight() {
  float val = analogRead(34);
  return map(val, 0, 4096*(3.3/5), 0, 100);
}

static HumiditySensorData getHumiditySensorData() {
  int i, j;
  int duree[42];
  unsigned long pulse;
  byte data[5];
  int broche = 16;
  
  pinMode(broche, OUTPUT_OPEN_DRAIN);
  digitalWrite(broche, HIGH);
  delay(250);
  digitalWrite(broche, LOW);
  delay(20);
  digitalWrite(broche, HIGH);
  delayMicroseconds(40);
  pinMode(broche, INPUT_PULLUP);
  
  while (digitalRead(broche) == HIGH);
  i = 0;

  do {
        pulse = pulseIn(broche, HIGH);
        duree[i] = pulse;
        i++;
  } while (pulse != 0);
 
  if (i != 42) 
    Serial.printf(" Erreur timing \n"); 

  for (i=0; i<5; i++) {
    data[i] = 0;
    for (j = ((8*i)+1); j < ((8*i)+9); j++) {
      data[i] = data[i] * 2;
      if (duree[j] > 50) {
        data[i] = data[i] + 1;
      }
    }
  }

  if ( (data[0] + data[1] + data[2] + data[3]) != data[4] ) 
    Serial.println(" Erreur checksum");

  HumiditySensorData result;
  result.humidity = data[0] + (data[1] / 256.0);
  result.temperature = data [2] + (data[3] / 256.0);
  return result;
  
}

static const char* getWindDirection() {
  static const char* directions[]     = {"N",   "NNE", "NE",  "NEE", "E",   "SEE", "SE",  "SSE", "S",   "SSW", "SW",  "SWW", "W",   "NWW", "NW",  "NNW"};
  static const float directionVals[]  = {0.81f, 0.67f, 1.90f, 1.80f, 3.14f, 2.57f, 2.83f, 2.14f, 2.42f, 1.18f, 1.36f, 0.15f, 0.18f, 0.08f, 0.47f, 0.28f};
  static const float marge = 0.02;


  float val = analogRead(35) / 4096.0 * 3.3f;

  for (int i = 0; i < sizeof(directions) / sizeof(char*); i++)
  {
    if (val >= directionVals[i] - marge && val <= directionVals[i] + marge)
    {
      return directions[i];
    }
  }
  return "UNKNOWN DIRECTION";
}

static float getWindSpeed() {
  return weatherMeterKit.getWindSpeed();
}

static float getRainfall() {
  return weatherMeterKit.getTotalRainfall();
}

static void updateSensorData() {
  // Only read sensors when needed (track last update times)
  static unsigned long lastLightUpdate = 0;
  static unsigned long lastHumidityUpdate = 0;
  static unsigned long lastPressureUpdate = 0;
  static unsigned long lastWindUpdate = 0;
  static unsigned long lastRainUpdate = 0;
  static unsigned long lastRainfallReset = millis();
  static float lastRainfall = 0;
  bool updatedValues = false;

  // Update rainfall every second
  if (millis() - lastRainUpdate >= 1000) {
    float currentRainfall = getRainfall();
    
    // Update total rainfall
    if (data.rainfall != currentRainfall) {
      data.rainfall = currentRainfall;
      updatedValues = true;
    }
    
    data.rainfallRate = (currentRainfall - lastRainfall) * 3600.0; // mm/h
    lastRainfall = currentRainfall;
    lastRainUpdate = millis();
    updatedValues = true;
  }

  // Reset rainfall counter every 24 hours (optional)
  if (millis() - lastRainfallReset > 86400000) { // 24h in ms
    weatherMeterKit.resetTotalRainfall();
    lastRainfallReset = millis();
  }

  // Update every 1 sec
  if (millis() - lastLightUpdate >= 1000) {
    float newLight = getLight();
    if (data.light != newLight) {
      data.light = newLight;
      updatedValues = true;
    }
    lastLightUpdate = millis();
  }

  // Update humidity/temp every 5 seconds
  if (millis() - lastHumidityUpdate >= 5000) {
    HumiditySensorData hData = getHumiditySensorData();
    if (data.humidity != hData.humidity) {
      data.humidity = hData.humidity;
      updatedValues = true;
    }
    if (data.temperature != hData.temperature) { 
      data.temperature = hData.temperature;
      updatedValues = true;
    }
    lastHumidityUpdate = millis();
  }

  // Update pressure every 3 seconds
  if (millis() - lastPressureUpdate >= 3000) {
    PressureSensorData pData = getPressureSensorData();
    if (data.pressure != pData.pressure) {
      data.pressure = pData.pressure;
      updatedValues = true;
    }
    // if (data.temperature != pData.temperature) {
    //   data.temperature = pData.temperature;
    //   updatedValues = true;
    // }
    lastPressureUpdate = millis();
  }

  // Update wind every 1 second (fast-changing)
  if (millis() - lastWindUpdate >= 1000) {
    const char* newWindDirection = getWindDirection();
    if (strcmp(data.windDirection, newWindDirection) != 0) {
      data.windDirection = newWindDirection;
      updatedValues = true;
    }

    float newWindSpeed = getWindSpeed();
    if (data.windSpeed != newWindSpeed) {
      data.windSpeed = newWindSpeed;
      updatedValues = true;
    }
    lastWindUpdate = millis();
  }

  if (updatedValues) {
    displaySensorData();
  }
}

void displaySensorData() {
  Serial.println(F("\n====== Sensor Data ======"));
  
  Serial.print(F("Light: "));
  Serial.print(data.light);
  Serial.println(F("%"));

  Serial.print(F("Humidity: "));
  Serial.print(data.humidity);
  Serial.println(F("%"));

  Serial.print(F("Pressure: "));
  Serial.print(data.pressure);
  Serial.println(F(" Pa"));

  Serial.print(F("Temperature: "));
  Serial.print(data.temperature);
  Serial.println(F(" °C"));

  Serial.print(F("Wind: "));
  Serial.print(data.windSpeed);
  Serial.print(F(" km/h from "));
  Serial.println(data.windDirection);

  Serial.print(F("Rainfall: "));
  Serial.print(data.rainfall, 2);
  Serial.println(F(" mm/h"));

  Serial.println(F("======================"));
}

void loop() {
  updateSensorData();
}
