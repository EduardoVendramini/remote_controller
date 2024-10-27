#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
const char *pin = "1234";
String slaveName = "HC-06-MODULO5";

LiquidCrystal_I2C lcd(0x3F, 16, 2);

String incomingMessage = "";

void setup()
{
  Serial.begin(9600);

  SerialBT.begin("ESP32", true);
  SerialBT.setPin(pin);

  //while (!SerialBT.connect(slaveName))
  //{
  //  Serial.println("Trying to connect to" + slaveName);
  //}
  //Serial.println("Bluetooth connected");

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Waiting input...");
}

void loop()
{
  if (SerialBT.available())
  {
    incomingMessage = SerialBT.readString();

    Serial.println(incomingMessage);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(incomingMessage);

    SerialBT.println(incomingMessage);
  }
}