#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>

#define GENERIC_ACCESS "00001800-0000-1000-8000-00805f9b34fb"
#define GENERIC_ATTRIBUTE "00001801-0000-1000-8000-00805F9B34FB"
#define DEVICE_INFORMATION "0000180A-0000-1000-8000-00805F9B34FB"

#define SERVICE_REFERENCE_INPUT "99FA0030-338A-1024-8A49-009C0215F78A"
#define REF_IN_ONE "99fa0031-338a-1024-8a49-009c0215f78a"

#define SERVICE_REFERENCE_OUTPUT "99FA0020-338A-1024-8A49-009C0215F78A"
#define REF_OUT_ONE "99fa0021-338a-1024-8a49-009c0215f78a" // height-speed
#define REF_OUT_MASK "99fa0029-338a-1024-8a49-009c0215f78a"
#define REF_OUT_DETECT_MASK "99fa002a-338a-1024-8a49-009c0215f78a"

#define SERVICE_DPG "99FA0010-338A-1024-8A49-009C0215F78A"
#define CHARACTERISTIC_DPG "99FA0011-338A-1024-8A49-009C0215F78A"

#define SERVICE_CONTROL "99FA0001-338A-1024-8A49-009C0215F78A"
#define CHARACTERISTIC_COMMAND "99FA0002-338A-1024-8A49-009C0215F78A"
#define CHARACTERISTIC_ERROR "99FA0003-338A-1024-8A49-009C0215F78A"

#pragma region Characteristics
BLECharacteristic *command = NULL;
BLECharacteristic *error = NULL;
BLECharacteristic *dpg = NULL;
BLECharacteristic *refOutONE = NULL;
BLECharacteristic *refOutMask = NULL;
BLECharacteristic *refOutDetectMask = NULL;
BLECharacteristic *refInONE = NULL;
#pragma endregion

#pragma region Globals
uint32_t lastHeight = 0;
uint32_t currentHeight = 5714;
uint32_t velocity = 100;  // (cm/100)/sec
bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer *pServer = NULL;
#pragma endregion

#pragma region Utilities
void printBytes(std::string value)
{
  if (value.length() > 0)
  {
    for (int i = 0; i < value.length(); i++)
    {
      if (i > 0)
      {
        Serial.print("-");
      }
      Serial.print(String(value[i], 16));
    }
  }
  else
  {
    Serial.print("NULL");
  }
  Serial.println();
}
#pragma endregion

#pragma region Callbacks
class CharacteristicSnoop : public BLECharacteristicCallbacks
{
protected:
  void onWrite(BLECharacteristic *pChar)
  {
    Serial.print("<<< Setting new value for ");
    Serial.print(pChar->toString().c_str());
    Serial.print("--> ");
    printBytes(pChar->getValue());
  }
  void onRead(BLECharacteristic *pChar)
  {
    Serial.print(">>> Getting characteristic: ");
    Serial.print(pChar->toString().c_str());
    Serial.print(" -->");
    printBytes(pChar->getValue());
  }
};

class CommandCallbacks : public CharacteristicSnoop
{
  void onWrite(BLECharacteristic *pChar)
  {
    std::string value = pChar->getValue();
    if (value.length() != 2) {
      CharacteristicSnoop::onWrite(pChar);
      return;
    }

    if (value[1] == 0x00)
    {
      // it might be a up/down command
      switch (value[0])
      {
      case 0x47:
        moveUp();
        break;
      case 0x46:
        moveDown();
        break;
      case 0xFF:
        stop();
        break;
      default:
        Serial.print("Unknown control command: ");
        printBytes(value);
      }
      return;
    }

    CharacteristicSnoop::onWrite(pChar);
  }

private:
  void moveUp()
  {
    uint8_t secondsElapsed = 1;
    currentHeight += velocity * secondsElapsed;  // don't worry about simulating time for now
    Serial.println("Move up commanded");
  }
  void moveDown()
  {
    uint8_t secondsElapsed = 1;
    currentHeight -= velocity * secondsElapsed;  // don't worry about simulating time for now
    Serial.println("Move down commanded");
  }
  void stop()
  {
    Serial.println("Stop commanded");
  }
};

class DpgCallbacks : public CharacteristicSnoop
{
  void onWrite(BLECharacteristic *pChar)
  {
    std::string value = pChar->getValue();
    std::string response;
    if (value.length() == 3 && value[0] == 0x7f && value[2] == 0x00)
    {
      char capabilities[] = {0x01, 0x02, 0xFC, 0x01};
      char userId[] = {0x01, 0x11, 0x00, 0xC6, 0x0F, 0xCA, 0x6A, 0x36, 0x34, 0x47, 0x60, 0xA0, 0xE1, 0xA3, 0x32, 0x56, 0x92, 0x6C, 0xE4};
      char deskOffset[] = {0x01, 0x03, 0x01, 0x66, 0x18};
      char reminderSetting[] = {0x01, 0x0B, 0x07, 0x37, 0x05, 0x32, 0x0A, 0x1E, 0x1E, 0x92, 0x47, 0xE4, 0x7B};
      char memPos1[] = {0x01, 0x07, 0x01, 0x88, 0x05, 0x59, 0x44, 0xE4, 0x7B};
      char memPos2[] = {0x01, 0x07, 0x01, 0x2A, 0x16, 0xFD, 0x6D, 0xE4, 0x7B};
      char memPos3[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00};
      char memPos4[] = {0x0B, 0x00};
      switch (value[1])
      {
      case 0x80:
        Serial.print("Char11 commanded to 0x80 ->");
        response = std::string(capabilities, sizeof(capabilities) / sizeof(capabilities[0]));
        ;
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x86:
        Serial.print("Char11 commanded to 0x86 ->");
        response = std::string(userId, sizeof(userId) / sizeof(userId[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x81:
        Serial.print("Char11 commanded to 0x81 ->");
        response = std::string(deskOffset, sizeof(deskOffset) / sizeof(deskOffset[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x88:
        Serial.print("Char11 commanded to 0x88 ->");
        response = std::string(reminderSetting, sizeof(reminderSetting) / sizeof(reminderSetting[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x89:
        Serial.print("Char11 commanded to 0x89 ->");
        response = std::string(memPos1, sizeof(memPos1) / sizeof(memPos1[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x8A:
        Serial.print("Char11 commanded to 0x8a ->");
        response = std::string(memPos2, sizeof(memPos2) / sizeof(memPos2[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x8B:
        Serial.print("Char11 commanded to 0x8b ->");
        response = std::string(memPos3, sizeof(memPos3) / sizeof(memPos3[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      case 0x8C:
        Serial.print("Char11 commanded to 0x8c ->");
        response = std::string(memPos4, sizeof(memPos4) / sizeof(memPos4[0]));
        Serial.print("notified with value: ");
        printBytes(response);
        pChar->setValue(response);
        pChar->notify();
        break;
      default:
        Serial.print("Hit default Chr11 case with value of: ");
        printBytes(value);
      }
      return;
    }
    if (value.length() >= 5 && value[0] == 0x7F && value[1] == 0x86 && value[2] == 0x80)
    {
      // 0x7F-86-80-XX... seems to all have the same response
      char response8680[] = {0x01, 0x00};
      Serial.print("Char11 commanded to 0x86-80 ->");
      response = std::string(response8680, sizeof(response8680) / sizeof(response8680[0]));
      Serial.print("notified with value: ");
      printBytes(response);
      pChar->setValue(response);
      pChar->notify();
      return;
    }
    // Nothing matched, so show the default snooper
    CharacteristicSnoop::onWrite(pChar);
  }
};

class DescriptorSnoop : public BLEDescriptorCallbacks
{
  void onRead(BLEDescriptor *pDesc)
  {
    Serial.print(">>> Getting DESCRIPTOR: ");
    Serial.print(pDesc->toString().c_str());
    Serial.print(" -->");
    uint8_t *val = pDesc->getValue();
    size_t sz = pDesc->getLength();
    printBytes(std::string(*val, sz));
  }
  void onWrite(BLEDescriptor *pDesc)
  {
    Serial.print("<<< Setting DESCRIPTOR value for ");
    Serial.print(pDesc->toString().c_str());
    Serial.print("--> ");
    uint8_t *val = pDesc->getValue();
    size_t sz = pDesc->getLength();
    printBytes(std::string(*val, sz));
  }
};

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("onConnect");
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("onDisconnect");
  }
};
#pragma endregion

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("Desk 8888");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

#pragma region ControlService

  // create control service
  BLEService *controlService = pServer->createService(SERVICE_CONTROL);

  // create control characteristic
  command = controlService->createCharacteristic(
      CHARACTERISTIC_COMMAND,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);
  command->setCallbacks(new CommandCallbacks());

  // create error characteristic with 2902 descriptor
  error = controlService->createCharacteristic(
      CHARACTERISTIC_ERROR,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  BLE2902 *errorDescriptor = new BLE2902();
  errorDescriptor->setNotifications(true);
  errorDescriptor->setCallbacks(new DescriptorSnoop());
  error->addDescriptor(errorDescriptor);

  error->setCallbacks(new CharacteristicSnoop());

  // start control service
  controlService->start();
  Serial.print("Descriptor handle for error characteristic (0003) = ");
  Serial.println(errorDescriptor->toString().c_str());
#pragma endregion

#pragma region DPGService

  // create DPG Service
  BLEService *dpgService = pServer->createService(SERVICE_DPG);

  // create characteristic 11
  dpg = dpgService->createCharacteristic(
      CHARACTERISTIC_DPG,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  BLE2902 *chr11Desc = new BLE2902();
  chr11Desc->setCallbacks(new DescriptorSnoop());
  dpg->addDescriptor(chr11Desc);
  dpg->setCallbacks(new DpgCallbacks());

  // start DPG service
  dpgService->start();

  Serial.print("Descriptor handle for DPG Characteristic (0011) = ");
  Serial.println(chr11Desc->toString().c_str());
#pragma endregion

#pragma region ServiceReferenceOutput

  // create ReferenceOutputService
  BLEService *refOutService = pServer->createService(SERVICE_REFERENCE_OUTPUT);

  // create ReferenceOutput-ONE (heightSpeed) characteristic
  refOutONE = refOutService->createCharacteristic(
      REF_OUT_ONE,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  BLE2902 *refOutONEDescriptor = new BLE2902();
  refOutONEDescriptor->setCallbacks(new DescriptorSnoop());
  refOutONE->addDescriptor(refOutONEDescriptor);
  refOutONE->setCallbacks(new CharacteristicSnoop());
  refOutONE->setValue(currentHeight);

  // create ReferenceOutput-Mask characteristic
  refOutMask = refOutService->createCharacteristic(
      REF_OUT_MASK,
      BLECharacteristic::PROPERTY_READ);
  refOutMask->setCallbacks(new CharacteristicSnoop());
  uint8_t refOutMaskInitialVal = 1;
  refOutMask->setValue(&refOutMaskInitialVal, 1); // actual desk starts with value of 1, required for handshaking

  // create ReferenceOutput-Mask-Detect characteristic
  refOutDetectMask = refOutService->createCharacteristic(
      REF_OUT_DETECT_MASK,
      BLECharacteristic::PROPERTY_READ);
  refOutDetectMask->setCallbacks(new CharacteristicSnoop());

  // start ReferenceOutputService
  refOutService->start();
  Serial.print("Descriptor handle for heightSpeedDesc = ");
  Serial.println(refOutONEDescriptor->toString().c_str());

#pragma endregion

#pragma region ReferenceInputService

  BLEService *refInService = pServer->createService(SERVICE_REFERENCE_INPUT);
  refInONE = refInService->createCharacteristic(
      REF_IN_ONE,
      BLECharacteristic::PROPERTY_WRITE_NR);
  refInONE->setCallbacks(new CharacteristicSnoop());

  refInService->start();

#pragma endregion

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(244);
  pAdvertising->setMaxInterval(510);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->addServiceUUID(SERVICE_CONTROL);
  // pAdvertising->addServiceUUID(SERVICE_DPG);
  // pAdvertising->addServiceUUID(SERVICE_REFERENCE_OUTPUT);
  // pAdvertising->addServiceUUID(SERVICE_REFERENCE_INPUT);
  BLEDevice::startAdvertising();
  Serial.println("Services defined and advertising started");
}

void loop()
{

  // notify changed values if deviceConnected
  if (deviceConnected)
  {
    // call ->setValue() and/or ->notify() here
    // update if height changed:
    if (currentHeight != lastHeight)
    {
      refOutONE->setValue(currentHeight);
      refOutONE->notify();
      lastHeight = currentHeight;
      Serial.println("Updated height and notified");
    }
    delay(10);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // let Bluetooth stack reset itself
    pServer->startAdvertising();
    Serial.println("Started advertising");
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    Serial.println("Just connected--");
    oldDeviceConnected = deviceConnected;
  }
}
