#include <Adafruit_DPS310.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "6751b732-f992-11ed-be56-0242ac120002"
#define CHARACTERISTIC_UUID_RX "6751b733-f992-11ed-be56-0242ac120002"
#define CHARACTERISTIC_UUID_TX "6751b734-f992-11ed-be56-0242ac120002"

Adafruit_DPS310 dps;

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
};

static float getLight();
static HumiditySensorData getHumiditySensorData();
static PressureSensorData getPressureSensorData();
static const char* getWindDirection();
static float getWindSpeed();
static void updateSensorData();


SensorData data;

bool deviceConnected = false;

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;

bool newDataFlag = false;
String displayString = ""; // À définir avec le format souhaité

// Déclarez cette structure si elle n'existe pas encore
struct ExtendedSensorData {
  float waterLevel = 0.0f;
  // Ajouter d'autres champs si nécessaire
};
ExtendedSensorData sensors;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.print("Received via BLE: ");
    Serial.println(value.c_str());
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);  // Attend que le port série soit prêt (utile sur certains PC)

  if (!dps.begin_I2C()) {
    Serial.println("Erreur : capteur DPS310 non détecté !");
    while (1); // Boucle infinie si le capteur n'est pas trouvé
  }

  Serial.println("Capteur DPS310 détecté.");

  BLEDevice::init("Mark I");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();
}

static PressureSensorData getPressureSensorData() {
  sensors_event_t tempEvent, pressureEvent;

  // Lire température et pression
  dps.getEvents(&tempEvent, &pressureEvent);

  delay(1000);

  PressureSensorData result;
  result.pressure = pressureEvent.pressure * 100; // hPa → Pa
  result.temperature = tempEvent.temperature;
  return result;
}

static float getLight() {
  float val = analogRead(34);
  delay(1000);
  return map((int)val, 0, (int)(4096 * (3.3 / 5.0)), 0, 100);
}

static HumiditySensorData getHumiditySensorData() {
  int i, j;
  int duree[42];
  unsigned long pulse;
  byte data[5];
  int broche = 16;

  delay(2000);
  
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
  // Serial.printf("Raw analog: %d, Normalized: %.2f\n", analogRead(35), val);
  delay(1000);

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
  static bool oldClick = false;
  static unsigned long start = millis();
  static unsigned long lastActiveTime = millis();
  static int clicks = 0;

  float val = analogRead(27); // Read wind sensor

  // Edge detection (same as before)
  if (oldClick && val == 0) {
    clicks++;
    oldClick = false;
    lastActiveTime = millis();
  } 
  else if (!oldClick && val > 0) {
    oldClick = true;
    lastActiveTime = millis();
  }

  // Calculate speed in km/h (calibrated to your sensor)
  unsigned long elapsed = millis() - start;
  if (elapsed > 0) {
    float clicksPerSecond = clicks / (elapsed / 1000.0f); // Convert ms → seconds
    float windSpeed = clicksPerSecond * 2.4f; // Your calibration factor
    return windSpeed;
  }

  // Auto-reset if no wind for 3 seconds
  if (millis() - lastActiveTime > 3000) {
    return 0.0f;
  }

  // Periodic reset (every 30 seconds to prevent overflow)
  if (elapsed > 30000) {
    start = millis();
    clicks = 0;
  }

  return 0.0f;
}

static void updateSensorData() {
  // Only read sensors when needed (track last update times)
  static unsigned long lastLightUpdate = 0;
  static unsigned long lastHumidityUpdate = 0;
  static unsigned long lastPressureUpdate = 0;
  static unsigned long lastWindUpdate = 0;
  bool updatedValues = false;

  // Update light every 2 seconds (example interval)
  if (millis() - lastLightUpdate >= 2000) {
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
  Serial.println(F("\n====== Sensor Data ======"));  // F() stores string in PROGMEM to save RAM
  
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

  Serial.println(F("======================"));
}

static void water_level() {
  static long timer = millis();
  if (millis() - timer > 250)
  {
    int val = digitalRead(23);
    if (val == 0)
    {
      newDataFlag = true;
      sensors.waterLevel += 0.2794f;
      timer = millis();
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // HumiditySensorData humiditySensorData = getHumiditySensorData();
  // float humidity = humiditySensorData.humidity;
  // float temperature = humiditySensorData.temperature;
  // Serial.printf(" humidity = %4.0f \%%  Temperature = %4.2f degreC \n", humidity, temperature);


  // PressureSensorData data = getPressureSensorData();
  // Serial.print("Température : ");
  // Serial.print(data.temperature);
  // Serial.println(" °C");

  // Serial.print("Pression : ");
  // Serial.print(data.pressure );  // hPa → Pa
  // Serial.println(" Pa");

  // const float windSpeed = getWindSpeed();
  // Serial.print("windSpeed: ");
  // Serial.println(windSpeed);

  // const float windSpeed = getWindSpeed();
  // Serial.print("windSpeed: ");
  // Serial.println(windSpeed);
  
  updateSensorData();

  if (deviceConnected and newDataFlag) {
    pTxCharacteristic->setValue(displayString);
    pTxCharacteristic->notify();
    newDataFlag = false;
  }
}
