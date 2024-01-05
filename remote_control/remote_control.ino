#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);

float kp, kd, ki = 0.0;
float m1, m2 = 0.0;
float gyr, ref = 0.0;

void setup() {
  lcd.init(); 
  lcd.backlight();
  lcd.clear();

  lcd.print("Hello user");
  delay(2000); 
  lcd.clear(); 

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
}

void loop() {
  int openLoopState = digitalRead(3);
  int setupState = digitalRead(4);

  
  if (openLoopState == HIGH) {
    setMotors(A1, A3);
  } else if (setupState == HIGH){
    setup(A1, A3);
  } else{
    setGains(A1, A2, A3);
  }
    
  setDisturbance(2);
}


void setDisturbance(int pinDisturbance){
  int digitalState = digitalRead(pinDisturbance);
  
  lcd.setCursor(12, 0); 

  if (digitalState == HIGH) {
    lcd.print("pOn ");
  } else if (digitalState == LOW){
    lcd.print("pOff");
  }
}


void setGains(int pinKp, int pinKd, int pinKi) {
  int potKp = analogRead(pinKp);
  int potKd = analogRead(pinKd);
  int potKi = analogRead(pinKi);

  kp = map(potKp, 0, 1023, 0, 100) / 1000.0;
  kd = map(potKd, 0, 1023, 0, 100) / 1000.0;
  ki = map(potKi, 0, 1023, 0, 100) / 1000.0;
  
  //lcd.clear(); // Clear the LCD before printing new values  
  lcd.setCursor(0, 0);
  lcd.print("kp kd ki   ");

  lcd.setCursor(0, 1); // set cursor to first column second row
  lcd.print(kp);
  lcd.print(" ");
  lcd.print(kd);
  lcd.print(" ");
  lcd.print(ki);
}

void setMotors(int pinM1, int pinM2){
  lcd.setCursor(0, 0); 
  lcd.print("M1 and m2   ");

  int potM1 = analogRead(pinM1);
  int potM2 = analogRead(pinM2);

  m1 = map(potM1, 0, 1023, 1000, 1300);
  m2 = map(potM2, 0, 1023, 1000, 1300);

  lcd.setCursor(0, 1);
  lcd.print(m1);
  lcd.print("  ");
  lcd.print(m2);
}

void setup(int pinGyr, int pinRef){
  lcd.setCursor(0, 0);
  lcd.print("Gyr and ref  ");

  int potGyr = analogRead(pinGyr);
  int potRef = analogRead(pinRef);

  gyr = map(potGyr, 0, 1023, 90, 100) / 100.0;
  ref = map(potRef, 0, 1023, -90, 90)/ 1.0;

  lcd.setCursor(0, 1);
  lcd.print(gyr);
  lcd.print("     ");
  lcd.print(ref);
}
