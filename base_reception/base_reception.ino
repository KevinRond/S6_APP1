#include "BLEDevice.h"

#define SERVICE_UUID "6751b732-f992-11ed-be56-0242ac120002"
#define CHARACTERISTIC_UUID_RX "6751b733-f992-11ed-be56-0242ac120002"
#define CHARACTERISTIC_UUID_TX "6751b734-f992-11ed-be56-0242ac120002"

static BLEAdvertisedDevice* myDevice;
static bool doConnect = false;
static bool doScan = false;
static bool connected = false;

static BLERemoteCharacteristic* pRemoteCharacteristicTX = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicRX = nullptr;

// --- Callback de notification depuis la TX ---
void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notification reçue: ");
  Serial.write(pData, length);
  Serial.println();
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Périphérique trouvé: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}

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

  BLERemoteService* pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.println("Service non trouvé");
    pClient->disconnect();
    return false;
  }

  // Obtenir TX
  pRemoteCharacteristicTX = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID_TX));
  if (pRemoteCharacteristicTX == nullptr) {
    Serial.println("Caractéristique TX non trouvée");
    pClient->disconnect();
    return false;
  }

  // Obtenir RX
  pRemoteCharacteristicRX = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID_RX));
  if (pRemoteCharacteristicRX == nullptr) {
    Serial.println("Caractéristique RX non trouvée");
    pClient->disconnect();
    return false;
  }

  // Enregistrement des notifications sur TX
  if (pRemoteCharacteristicTX->canNotify()) {
    pRemoteCharacteristicTX->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("Mark 2");

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

  if (connected && pRemoteCharacteristicRX != nullptr) {
    String newValue = "Time: " + String(millis() / 1000);
    pRemoteCharacteristicRX->writeValue(newValue.c_str(), newValue.length());
    Serial.println("Donnée envoyée: " + newValue);
  }
  else if (doScan) {
    BLEDevice::getScan()->start(0);  // Recommence le scan sans durée limite
  }

  delay(1000);
}