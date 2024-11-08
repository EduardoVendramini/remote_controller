#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PIN_DISTURBANCE (2)         // Perturbação
#define PIN_OPEN_LOOP (3)           // MA
#define PIN_SETUP (4)               // Prog
#define PIN_POTENTIOMETER_SETUP (5) // Ajuste Pot.
#define POTENTIOMETER_TOP (A1)      // A1
#define POTENTIOMETER_MID (A2)      // A2
#define POTENTIOMETER_BOTTOM (A3)   // A3

LiquidCrystal_I2C lcd(0x3F, 16, 2);

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

void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.print("Olaaaa!!!!!");
  delay(2000);
  lcd.clear();

  pinMode(PIN_DISTURBANCE, INPUT_PULLUP);
  pinMode(PIN_OPEN_LOOP, INPUT_PULLUP);
  pinMode(PIN_SETUP, INPUT_PULLUP);
  pinMode(PIN_POTENTIOMETER_SETUP, INPUT_PULLUP);
}

void loop()
{
int openLoopState = digitalRead(PIN_OPEN_LOOP);
int setupState = digitalRead(PIN_SETUP);

  if (openLoopState == HIGH)
  {
    setMotors();
  }
  else if (setupState == HIGH)
  {
    setupGyrRef();
  }
  else
  {
    setGains();
  }

  setDisturbance();
}

void setDisturbance()
{
  disturbanceState = digitalRead(PIN_DISTURBANCE);

  lcd.setCursor(12, 0);

  if (disturbanceState == HIGH)
  {
    lcd.print("pOn ");
  }
  else if (disturbanceState == LOW)
  {
    lcd.print("pOff");
  }
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

  potentiometerSetupState = digitalRead(PIN_POTENTIOMETER_SETUP);

  if (potentiometerSetupState == HIGH)
  {
    lcd.print("Posicionar pots.");
  }
  else if (potentiometerSetupState == LOW)
  {
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
}

void setMotors()
{
  // write in the first row of lcd
  lcd.setCursor(0, 0);
  lcd.print("M1 and m2   ");

  potentiometerSetupState = digitalRead(PIN_POTENTIOMETER_SETUP);

  // write in the second row of lcd
  lcd.setCursor(0, 1);

  potM1 = analogRead(POTENTIOMETER_TOP);
  potM2 = analogRead(POTENTIOMETER_BOTTOM);

  if (potentiometerSetupState == HIGH)
  {
    lcd.print("Posicionar pots.");
  }
  else if (potentiometerSetupState == LOW)
  {
    // m1
    if (potM1 > lastPotM1 + 5)
    {
      m1 += 5;
      if (m1 > 1300)
      {
        m1 = 1300;
      }
    }
    else if (potM1 < lastPotM1 - 5)
    {
      m1 -= 5;
      if (m1 < 1100)
      {
        m1 = 1100;
      }
    }

    // m2
    if (potM2 > lastPotM2 + 5)
    {
      m2 += 5;
      if (m2 > 1300)
      {
        m2 = 1300;
      }
    }
    else if (potM2 < lastPotM2 - 5)
    {
      m2 -= 5;
      if (m2 < 1100)
      {
        m2 = 1100;
      }
    }

    lastPotM1 = potM1;
    lastPotM2 = potM2;

    lcd.print(m1);
    lcd.print("  ");
    lcd.print(m2);
    lcd.print("    ");
  }
}

void setupGyrRef()
{
  // write in the first row of the lcd
  lcd.setCursor(0, 0);
  lcd.print("Gyr and ref  ");

  // write in the second row of the lcd
  lcd.setCursor(0, 1);

  potentiometerSetupState = digitalRead(PIN_POTENTIOMETER_SETUP);

  if (potentiometerSetupState == HIGH)
  {
    lcd.print("Posicionar pots.");
  }
  else if (potentiometerSetupState == LOW)
  {
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
}