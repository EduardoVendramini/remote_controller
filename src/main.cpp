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
void bluetoothScan();
void bluetoothConnect();

// LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Buttons
#define PIN_DISTURBANCE 15         // Perturbação
#define PIN_OPEN_LOOP 3           // MA
#define PIN_SETUP 4               // Prog
#define PIN_POTENTIOMETER_SETUP 5 // Ajuste Pot.
#define POTENTIOMETER_TOP 33      // Equivalente ao A1 (ADC1_CH5)
#define POTENTIOMETER_MID 32      // Equivalente ao A2 (ADC1_CH4)
#define POTENTIOMETER_BOTTOM 35   // Equivalente ao A3 (ADC1_CH7)
#define MAX_MOTOR_SPEED 2000
#define MIN_MOTOR_SPEED 1000

float kp = 0.0, kd = 0.0, ki = 0.0;
float m1 = 1200, m2 = 1200;
float ref = 0.0;
float gyr = 0.99;

int openLoopState = digitalRead(PIN_OPEN_LOOP);
int setupState = digitalRead(PIN_SETUP);
int disturbanceState = digitalRead(PIN_DISTURBANCE);
int potentiometerSetupState = digitalRead(PIN_POTENTIOMETER_SETUP);

int potKp = analogRead(POTENTIOMETER_TOP), potKd = analogRead(POTENTIOMETER_MID), potKi = analogRead(POTENTIOMETER_BOTTOM);
int potM1 = analogRead(POTENTIOMETER_TOP), potM2 = analogRead(POTENTIOMETER_BOTTOM);
int potGyr = analogRead(POTENTIOMETER_TOP), potRef = analogRead(POTENTIOMETER_BOTTOM);
int lastPotKp = analogRead(POTENTIOMETER_TOP), lastPotKd = analogRead(POTENTIOMETER_MID), lastPotKi = analogRead(POTENTIOMETER_BOTTOM);
int lastPotM1 = analogRead(POTENTIOMETER_TOP), lastPotM2 = analogRead(POTENTIOMETER_BOTTOM);
int lastPotGyr = analogRead(POTENTIOMETER_TOP), lastPotRef = analogRead(POTENTIOMETER_BOTTOM);

void setDisturbance();
void setGains();
void setMotors();
void setupGyrRef();
void potentiometerSetup();

// Serial output
String incomingMessage = "";

void setup()
{
  // Serial
  Serial.begin(9600);

  // Bluetooth
  SerialBT.begin("ESP32", true);
  // bluetoothScan();
  // bluetoothConnect();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello.");

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
    potentiometerSetup();

  else
  {
    if (openLoopState == LOW)
      setMotors();
    else if (setupState == LOW)
      setupGyrRef();
    else
      setGains();
  }

  setDisturbance();

  // if (Serial.available())
  //{
  //   incomingMessage = Serial.readString();
  //
  //  Serial.println(incomingMessage);
  //  SerialBT.println(incomingMessage);
  //
  //  lcd.clear();
  //  lcd.setCursor(0, 0);
  //  lcd.print(incomingMessage);
  //
  //  incomingMessage = "";
  //}
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

void setDisturbance()
{
  disturbanceState = digitalRead(PIN_DISTURBANCE);

  lcd.setCursor(12, 0);

  if (disturbanceState == LOW)
    lcd.print("pOn ");

  else
    lcd.print("pOff");
}

void setGains()
{
  // lcd.clear(); // Clear the LCD before printing new values
  lcd.setCursor(0, 0);
  lcd.print("kp kd ki   ");

  // set cursor to first column second row
  potKp = analogRead(POTENTIOMETER_TOP);
  potKd = analogRead(POTENTIOMETER_MID);
  potKi = analogRead(POTENTIOMETER_BOTTOM);

  lcd.setCursor(0, 1);

  // kp
  if (potKp > lastPotKp + 5)
  {
    kp += 0.01;
    if (kp > 0.90)
    {
      kp = 0.90;
    }
  }
  else if (potKp < lastPotKp - 5)
  {
    kp -= 0.01;
    if (kp < 0)
    {
      kp = 0.00;
    }
  }

  // kd
  if (potKd > lastPotKd + 5)
  {
    kd += 0.01;
    if (kd > 0.90)
    {
      kd = 0.90;
    }
  }
  else if (potKd < lastPotKd - 5)
  {
    kd -= 0.01;
    if (kd < 0.0)
    {
      kd = 0.0;
    }
  }

  // ki
  if (potKi > lastPotKi + 5)
  {
    ki += 0.01;
    if (ki > 0.90)
    {
      ki = 0.90;
    }
  }
  else if (potKi < lastPotKi - 5)
  {
    ki -= 0.01;
    if (ki < 0)
    {
      ki = 0.00;
    }
  }

  lastPotKp = potKp;
  lastPotKd = potKd;
  lastPotKi = potKi;

  lcd.print(kp);
  lcd.print(" ");
  lcd.print(kd);
  lcd.print(" ");
  lcd.print(ki);
  lcd.print("  ");
}

void setMotors()
{
  // write in the first row of lcd
  lcd.setCursor(0, 0);
  lcd.print("M1 and m2   ");

  // write in the second row of lcd
  lcd.setCursor(0, 1);

  potM1 = analogRead(POTENTIOMETER_TOP);
  potM2 = analogRead(POTENTIOMETER_BOTTOM);

  // m1
  if (potM1 > lastPotM1 + 5)
  {
    m1 += 5;
    if (m1 > MAX_MOTOR_SPEED)
    {
      m1 = MAX_MOTOR_SPEED;
    }
  }
  else if (potM1 < lastPotM1 - 5)
  {
    m1 -= 5;
    if (m1 < MIN_MOTOR_SPEED)
    {
      m1 = MIN_MOTOR_SPEED;
    }
  }

  // m2
  if (potM2 > lastPotM2 + 5)
  {
    m2 += 5;
    if (m2 > MAX_MOTOR_SPEED)
    {
      m2 = MAX_MOTOR_SPEED;
    }
  }
  else if (potM2 < lastPotM2 - 5)
  {
    m2 -= 5;
    if (m2 < MIN_MOTOR_SPEED)
    {
      m2 = MIN_MOTOR_SPEED;
    }
  }

  lastPotM1 = potM1;
  lastPotM2 = potM2;

  lcd.print(m1);
  lcd.print("  ");
  lcd.print(m2);
  lcd.print("    ");
}

void setupGyrRef()
{
  // write in the first row of the lcd
  lcd.setCursor(0, 0);
  lcd.print("Gyr and ref  ");

  // write in the second row of the lcd
  lcd.setCursor(0, 1);

  potGyr = analogRead(POTENTIOMETER_TOP);
  potRef = analogRead(POTENTIOMETER_BOTTOM);

  // Gyr
  if (potGyr > lastPotGyr + 5)
  {
    gyr += 0.01;
    if (gyr > 1.0)
    {
      gyr = 1.0;
    }
  }
  else if (potGyr < lastPotGyr - 5)
  {
    gyr -= 0.01;
    if (gyr < 0.9)
    {
      gyr = 0.9;
    }
  }

  // Ref
  if (potRef > lastPotRef + 5)
  {
    ref += 5;
    if (ref > 180)
    {
      ref = 180;
    }
  }
  else if (potRef < lastPotRef - 5)
  {
    ref -= 5;
    if (ref < -180)
    {
      ref = -180;
    }
  }

  lastPotGyr = potGyr;
  lastPotRef = potRef;

  lcd.print(gyr);
  lcd.print("     ");
  lcd.print(ref);
  lcd.print("     ");
}

void potentiometerSetup()
{
  lcd.setCursor(0, 1);
  lcd.print("Posicionar pots.");
}
