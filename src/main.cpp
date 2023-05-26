#include <Arduino.h>

#define ON1 PA5
#define ON2 PA6
#define ON3 PA7
#define OFF1 PB0
#define OFF2 PB1
#define OFF3 PB10

// put function declarations here:
int myFunction(int, int);

void vsatON()
{
    digitalWrite(ON1, HIGH);
    delay(500);
    digitalWrite(ON1, LOW);
    delay(500);
}

void vsatOFF()
{
    digitalWrite(OFF1, HIGH);
    delay(500);
    digitalWrite(OFF1, LOW);
    delay(500);
}

void btsON()
{
    digitalWrite(ON2, HIGH);
    delay(500);
    digitalWrite(ON2, LOW);
    delay(500);
}

void btsOFF()
{
    digitalWrite(OFF2, HIGH);
    delay(500);
    digitalWrite(OFF2, LOW);
    delay(500);
}

void otherON()
{
    digitalWrite(ON3, HIGH);
    delay(500);
    digitalWrite(ON3, LOW);
    delay(500);
}

void otherOFF()
{
    digitalWrite(OFF3, HIGH);
    delay(500);
    digitalWrite(OFF3, LOW);
    delay(500);
}

void setup() {
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  pinMode(ON1, OUTPUT);
  pinMode(ON2, OUTPUT);
  pinMode(ON3, OUTPUT);
  pinMode(OFF1, OUTPUT);
  pinMode(OFF2, OUTPUT);
  pinMode(OFF3, OUTPUT);
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello");
  delay(200);
  Serial.println("End");
  // vsatON();
  // vsatOFF();
  // btsON();
  // btsOFF();
  // otherON();
  // otherOFF();
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}