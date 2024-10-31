#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>

#define BT_DISCOVER_TIME 10000

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

LiquidCrystal_I2C lcd(0x3F, 16, 2);

String incomingMessage = "";

BluetoothSerial SerialBT;
uint8_t address[6] = {0x98, 0xDA, 0x60, 0x00, 0xEB, 0x2E}; // MAC Address of the slave, got from scanning
void bluetoothScan();
void bluetoothConnect(uint8_t address[]);

void setup()
{
  // Serial
  Serial.begin(9600);

  // Bluetooth
  SerialBT.begin("ESP32", true);
  // bluetoothScan();
  bluetoothConnect(address);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Waiting input...");
}

void loop()
{
  if (Serial.available())
  {
    incomingMessage = Serial.readString();

    Serial.println(incomingMessage);
    SerialBT.println(incomingMessage);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(incomingMessage);

    incomingMessage = "";
  }
}

void bluetoothScan()
{
  Serial.println("Starting discover...");
  BTScanResults *pResults = SerialBT.discover(BT_DISCOVER_TIME);
  if (pResults)
    pResults->dump(&Serial);
  else
    Serial.println("Error on BT Scan, no result!");
}

void bluetoothConnect(uint8_t address[])
{
  for (int i = 0; i < 3; i++)
  {
    Serial.println("Connecting...");
    if (SerialBT.connect(address))
    {
      Serial.println("Connected");
      return;
    }
    else
    {
      Serial.println("Failed to connect");
    }
  }
}