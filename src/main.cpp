#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>

// Bluetooth
#define BT_DISCOVER_TIME 10000
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
uint8_t address[6] = {0x98, 0xDA, 0x60, 0x00, 0xEB, 0x2E}; // MAC Address of the slave, got from scanning
// uint8_t address[6] = {0x98, 0xDA, 0x60, 0x00, 0xD3, 0x2A}; // From home
// void bluetoothScan();
void bluetoothConnect();
unsigned long lastBluetoothSendTime = 0;
const unsigned long bluetoothInterval = 1200;

// LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Buttons
#define PIN_DISTURBANCE 15        // Perturbação
#define PIN_OPEN_LOOP 16          // MA
#define PIN_SETUP 4               // Prog
#define PIN_POTENTIOMETER_SETUP 5 // Ajuste Pot.
#define POTENTIOMETER_TOP 33      // Equivalente ao A1 (ADC1_CH5)
#define POTENTIOMETER_MID 32      // Equivalente ao A2 (ADC1_CH4)
#define POTENTIOMETER_BOTTOM 35   // Equivalente ao A3 (ADC1_CH7)
#define MAX_MOTOR_SPEED 2000
#define MIN_MOTOR_SPEED 1000
#define MAX_KP 0.90
#define MAX_KD 0.90
#define MAX_KI 0.90
#define MAX_GYR 1.0
#define MIN_GYR 0.9
#define MAX_REF 90
#define MIN_REF -90

float kp = 0.0, kd = 0.0, ki = 0.0;
int m1 = 1200, m2 = 1200, mIncrement = 5;
float ref = 0.0, refIncrement = 5.0;
float gyr = 0.99, centesimalFloatIncrement = 0.01;

int incrementMultiplier = 1, incrementMultiplierCounter = 1;
unsigned long timeNow = 0;

int openLoopState = 0, setupState = 0, disturbanceState = 0, potentiometerSetupState = 0, automaticState = 0;
int potKp, potKd, potKi,
    potM1, potM2,
    potGyr, potRef,
    lastPotKp, lastPotKd, lastPotKi,
    lastPotM1, lastPotM2,
    lastPotGyr, lastPotRef = 0;
int potOffset = 100;

void setDisturbance();
void setGains();
void setMotors();
void setupGyrRef();
void potentiometerSetup();
void sendBluetoothMessage();

// Serial output
String outcomingMessage = "";

void setup()
{
  // Serial
  Serial.begin(9600);
  Serial.println("Starting.");

  // LCD
  lcd.init();
  lcd.backlight();

  // Bluetooth
  SerialBT.begin("ESP32", true);

  // bluetoothScan();
  bluetoothConnect();

  // Buttons
  pinMode(PIN_DISTURBANCE, INPUT_PULLUP);
  pinMode(PIN_OPEN_LOOP, INPUT_PULLUP);
  pinMode(PIN_SETUP, INPUT_PULLUP);
  pinMode(PIN_POTENTIOMETER_SETUP, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_POTENTIOMETER_SETUP), potentiometerSetup, LOW);
}

void loop()
{
  openLoopState = digitalRead(PIN_OPEN_LOOP);
  setupState = digitalRead(PIN_SETUP);
  potentiometerSetupState = digitalRead(PIN_POTENTIOMETER_SETUP);

  if (potentiometerSetupState == LOW)
  {
    potentiometerSetup();
  }

  else if (openLoopState == LOW)
  {
    automaticState = 0;
    setMotors();
  }
  else if (setupState == LOW)
  {
    setupGyrRef();
    automaticState = 1;
  }
  else
  {
    setGains();
    automaticState = 1;
  }

  setDisturbance();

  sendBluetoothMessage();
}

void sendBluetoothMessage()
{
  // kp, ki, kd, m1, m2, gyr, ref, automaticState, disturbanceState
  outcomingMessage = String(kp) + "," + String(ki) + "," + String(kd) + "," + String(m1) + "," + String(m2) + "," +
                     String(gyr) + "," + String(ref) + "," + String(automaticState) + "," + String(disturbanceState);
  // outcomingMessage = "p" + String(kp) + ",d" + String(kd) + ",i" + String(ki) + ",y" + String(int(m1)) + ",t" + String(int(m2)) + ",g" +
  //                    String(gyr) + ",r" + String(ref) + ",a" + String(automaticState) + ",b" + String(disturbanceState);

  if (millis() - lastBluetoothSendTime > bluetoothInterval)
  {
    SerialBT.println(outcomingMessage);
    Serial.println(outcomingMessage);
    lastBluetoothSendTime = millis();
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

void bluetoothConnect()
{

  for (int i = 0; i < 3; i++)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting...");
    Serial.println("Connecting...");
    if (SerialBT.connect(address))
    {
      lcd.setCursor(0, 0);
      lcd.print("Connected");
      Serial.println("Connected");
      return;
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connection");
      lcd.setCursor(0, 1);
      lcd.print("failed");
      Serial.println("Failed to connect");
      delay(1000);
    }
  }
}

void setDisturbance()
{
  disturbanceState = !digitalRead(PIN_DISTURBANCE); // pull-up

  lcd.setCursor(12, 0);

  if (disturbanceState == HIGH)
    lcd.print("pOn ");

  else
    lcd.print("pOff");
}

void setGains()
{
  lcd.setCursor(0, 0);
  lcd.print("kp kd ki   ");

  potKp = analogRead(POTENTIOMETER_TOP);
  potKd = analogRead(POTENTIOMETER_MID);
  potKi = analogRead(POTENTIOMETER_BOTTOM);

  // kp
  if (potKp > lastPotKp + potOffset)
  {
    kp += centesimalFloatIncrement;
    if (kp > MAX_KP)
    {
      kp = MAX_KP;
    }
  }
  else if (potKp < lastPotKp - potOffset)
  {
    kp -= centesimalFloatIncrement;
    if (kp < 0)
    {
      kp = 0.00;
    }
  }

  // kd
  if (potKd > lastPotKd + potOffset)
  {
    kd += centesimalFloatIncrement;
    if (kd > MAX_KD)
    {
      kd = MAX_KD;
    }
  }
  else if (potKd < lastPotKd - potOffset)
  {
    kd -= centesimalFloatIncrement;
    if (kd < 0.0)
    {
      kd = 0.0;
    }
  }

  // ki
  if (potKi > lastPotKi + potOffset)
  {
    ki += centesimalFloatIncrement;
    if (ki > MAX_KI)
    {
      ki = MAX_KI;
    }
  }
  else if (potKi < lastPotKi - potOffset)
  {
    ki -= centesimalFloatIncrement;
    if (ki < 0)
    {
      ki = 0.00;
    }
  }

  lastPotKp = potKp;
  lastPotKd = potKd;
  lastPotKi = potKi;

  lcd.setCursor(0, 1);
  lcd.print(kp);
  lcd.print(" ");
  lcd.print(kd);
  lcd.print(" ");
  lcd.print(ki);
  lcd.print(" ");
  lcd.print(incrementMultiplierCounter);
}

void setMotors()
{
  // write in the first row of lcd
  lcd.setCursor(0, 0);
  lcd.print("M1 and m2   ");

  potM1 = analogRead(POTENTIOMETER_TOP);
  potM2 = analogRead(POTENTIOMETER_BOTTOM);

  // m1
  if (potM1 > lastPotM1 + potOffset)
  {
    m1 += mIncrement;
    if (m1 > MAX_MOTOR_SPEED)
    {
      m1 = MAX_MOTOR_SPEED;
    }
  }
  else if (potM1 < lastPotM1 - potOffset)
  {
    m1 -= mIncrement;
    if (m1 < MIN_MOTOR_SPEED)
    {
      m1 = MIN_MOTOR_SPEED;
    }
  }

  // m2
  if (potM2 > lastPotM2 + potOffset)
  {
    m2 += mIncrement;
    if (m2 > MAX_MOTOR_SPEED)
    {
      m2 = MAX_MOTOR_SPEED;
    }
  }
  else if (potM2 < lastPotM2 - potOffset)
  {
    m2 -= mIncrement;
    if (m2 < MIN_MOTOR_SPEED)
    {
      m2 = MIN_MOTOR_SPEED;
    }
  }

  lastPotM1 = potM1;
  lastPotM2 = potM2;

  // write in the second row of lcd
  lcd.setCursor(0, 1);
  lcd.print(m1);
  lcd.print("  ");
  lcd.print(m2);
  lcd.print("     ");
  lcd.print(incrementMultiplierCounter);
}

void setupGyrRef()
{
  // write in the first row of the lcd
  lcd.setCursor(0, 0);
  lcd.print("Gyr and ref ");

  potGyr = analogRead(POTENTIOMETER_TOP);
  potRef = analogRead(POTENTIOMETER_BOTTOM);

  // Gyr
  if (potGyr > lastPotGyr + potOffset)
  {
    gyr += centesimalFloatIncrement;
    if (gyr > MAX_GYR)
    {
      gyr = MAX_GYR;
    }
  }
  else if (potGyr < lastPotGyr - potOffset)
  {
    gyr -= centesimalFloatIncrement;
    if (gyr < MIN_GYR)
    {
      gyr = MIN_GYR;
    }
  }

  // Ref
  if (potRef > lastPotRef + potOffset)
  {
    ref += refIncrement;
    if (ref > MAX_REF)
    {
      ref = MAX_REF;
    }
  }
  else if (potRef < lastPotRef - potOffset)
  {
    ref -= refIncrement;
    if (ref < MIN_REF)
    {
      ref = MIN_REF;
    }
  }

  lastPotGyr = potGyr;
  lastPotRef = potRef;

  // write in the second row of the lcd
  lcd.setCursor(0, 1);
  lcd.print(gyr);
  lcd.print("    ");
  lcd.print(ref);
  lcd.print(" ");
  lcd.print(incrementMultiplierCounter);
  lcd.print("  ");
}

void potentiometerSetup()
{
  timeNow = millis();

  while (potentiometerSetupState == LOW)
  {
    lcd.setCursor(0, 1);
    lcd.print("Posicionar pots.");
    delay(20);
    potentiometerSetupState = digitalRead(PIN_POTENTIOMETER_SETUP);
  }

  if (millis() - timeNow < 500)
  {
    if (incrementMultiplierCounter < 3)
      incrementMultiplierCounter++;
    else
      incrementMultiplierCounter = 1;

    switch (incrementMultiplierCounter)
    {
    case 1:
      incrementMultiplier = 1;
      break;
    case 2:
      incrementMultiplier = 10;
      break;
    case 3:
      incrementMultiplier = 20;
      break;
    default:
      break;
    }

    mIncrement = 5 * incrementMultiplier;
    centesimalFloatIncrement = 0.01 * incrementMultiplier;
  }
}
