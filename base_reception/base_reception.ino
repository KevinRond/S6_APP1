#include "BLEDevice.h"

static BLEUUID serviceUUID("6751b732-f992-11ed-be56-0242ac120002");
static BLEUUID charUUID("6751b734-f992-11ed-be56-0242ac120002");

static BLEAdvertisedDevice* myDevice;
static bool doConnect = false;
static bool doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static bool connected = false;


class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Périphérique trouvé: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) { }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Déconnecté du serveur BLE.");
  }
};

bool connectToServer() {
  Serial.println("Connexion au périphérique BLE...");
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println("Connecté!");

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Service non trouvé");
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Caractéristique non trouvée");
    pClient->disconnect();
    return false;
  }

  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("Valeur lue : ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notification reçue: ");
  Serial.write(pData, length);
  Serial.println();
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connecté au serveur BLE.");
    } else {
      Serial.println("Échec de la connexion.");
    }
    doConnect = false;
  }

  if (connected) {
    String newValue = "Time: " + String(millis()/1000);
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    Serial.println("Donnée envoyée: " + newValue);
  }
  else if (doScan) {
    BLEDevice::getScan()->start(0);  // recommence le scan sans durée limite
  }

  delay(1000);
}